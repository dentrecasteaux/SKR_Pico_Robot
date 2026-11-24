#!/usr/bin/env bash
# Flash the SKR Pico over Wi-Fi, through the Raspberry Pi (24 Nov).
#
# Usage, from the repository root on the Mac, after `pio run`:
#   tools/flash-skr.sh                     # .pio/build/pico/firmware.uf2 to robot@r2w-pi.local
#   tools/flash-skr.sh -y                  # skip the "is it safe?" question
#   tools/flash-skr.sh path/to/other.uf2   # another image
#   R2W_PI=robot@192.168.1.50 tools/flash-skr.sh
#
# What it does:
#   1. checks the .uf2 is newer than every source file (a stale build has
#      caught us out before) and that the robot is not balancing;
#   2. copies the image to ~/r2w/firmware.uf2 on the Pi;
#   3. on the Pi: stops robotd and robot-web (they hold the serial port),
#      does the 1200-baud touch, which reboots the RP2040 into its USB
#      bootloader, then loads the image with picotool if it is installed,
#      otherwise copies it onto the RPI-RP2 drive (sudo mount);
#   4. waits for the serial port to come back, restarts the services and
#      checks the new firmware answers (ping, BAL_STATUS).
#
# Only flash with the robot off the floor or resting on its training wheels:
# the board resets and the motors are released mid-flash. If a bad build
# ever stops the USB port working, the 1200-baud touch can't reach it: hold
# BOOTSEL on the SKR while plugging in, then run this script again (it finds
# the RPI-RP2 drive directly) or copy the .uf2 by hand.
set -euo pipefail

ASSUME_YES=0
IMAGE=""
for arg in "$@"; do
  case "$arg" in
    -y|--yes) ASSUME_YES=1 ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) IMAGE="$arg" ;;
  esac
done

TARGET="${R2W_PI:-robot@r2w-pi.local}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-$ROOT/.pio/build/pico/firmware.uf2}"

if [[ ! -f "$IMAGE" ]]; then
  echo "flash-skr: no image at $IMAGE (run 'pio run' first)" >&2
  exit 1
fi

# A stale build: any source file newer than the image.
newer=$(find "$ROOT/src" "$ROOT/include" "$ROOT/platformio.ini" -newer "$IMAGE" -type f 2>/dev/null | head -3)
if [[ -n "$newer" ]]; then
  echo "flash-skr: $IMAGE is older than:" >&2
  echo "$newer" | sed 's/^/    /' >&2
  echo "Rebuild first ('pio run'), or pass the image explicitly to flash it anyway." >&2
  [[ "$IMAGE" == "$ROOT/.pio/build/pico/firmware.uf2" ]] && exit 1
fi

echo "==> Image: $IMAGE ($(wc -c < "$IMAGE" | tr -d ' ') bytes, built $(date -r "$IMAGE" '+%d %b %H:%M'))"

state=$(ssh "$TARGET" 'cd r2w && python3 robotdctl.py bal-status 2>/dev/null' | tr ' ' '\n' | sed -n 's/^BAL=//p' || true)
if [[ "$state" == "RUNNING" || "$state" == "ARMING" || "$state" == "DRY" ]]; then
  echo "flash-skr: the robot is balancing (BAL=$state). Stop it first." >&2
  exit 1
fi

if [[ "$ASSUME_YES" != 1 ]]; then
  read -r -p "Robot off the floor or on its training wheels? [y/N] " answer
  [[ "$answer" == y || "$answer" == Y ]] || { echo "Cancelled."; exit 1; }
fi

echo "==> Copying the image to $TARGET:~/r2w/firmware.uf2"
scp -q "$IMAGE" "$TARGET:r2w/firmware.uf2"

echo "==> Flashing on the Pi"
ssh "$TARGET" 'bash -s' <<'REMOTE'
set -u
cd ~/r2w
IMAGE="$HOME/r2w/firmware.uf2"
PATTERN='/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_*-if00'
LABEL=/dev/disk/by-label/RPI-RP2

restart() {
  systemctl --user start robotd robot-web
}
trap restart EXIT

systemctl --user stop robot-web robotd
sleep 0.5

if [ -e "$LABEL" ]; then
  echo "    already in the bootloader (RPI-RP2 present)"
else
  port=$(ls $PATTERN 2>/dev/null | head -1)
  if [ -z "$port" ]; then
    echo "    no SKR serial port and no RPI-RP2 drive: is the USB cable connected?" >&2
    exit 1
  fi
  echo "    1200-baud touch on $port"
  python3 - "$port" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 1200)
time.sleep(0.1)
s.close()
PY
fi

for _ in $(seq 1 50); do
  [ -e "$LABEL" ] && break
  sleep 0.2
done
if [ ! -e "$LABEL" ]; then
  echo "    the bootloader drive (RPI-RP2) did not appear within 10 s" >&2
  echo "    hold BOOTSEL on the SKR while plugging it in, then run this again" >&2
  exit 1
fi
echo "    bootloader drive found"

flashed=0
if command -v picotool >/dev/null && picotool info >/dev/null 2>&1; then
  echo "    loading with picotool"
  picotool load -x "$IMAGE" && flashed=1
fi
if [ "$flashed" = 0 ]; then
  mnt=$(mktemp -d)
  if sudo -n mount "$LABEL" "$mnt" 2>/dev/null; then
    echo "    copying onto RPI-RP2"
    sudo -n cp "$IMAGE" "$mnt/" && sync && flashed=1
    sudo -n umount "$mnt" 2>/dev/null || true   # the board reboots by itself
  else
    echo "    could not mount RPI-RP2 (needs passwordless sudo, or install picotool:" >&2
    echo "    sudo apt install picotool)" >&2
  fi
  rmdir "$mnt" 2>/dev/null || true
fi
[ "$flashed" = 1 ] || exit 1

echo "    waiting for the SKR to restart"
for _ in $(seq 1 75); do
  ls $PATTERN >/dev/null 2>&1 && break
  sleep 0.2
done
if ! ls $PATTERN >/dev/null 2>&1; then
  echo "    the SKR serial port did not come back within 15 s" >&2
  exit 1
fi
sleep 1
restart
trap - EXIT
sleep 2
echo "    robotd:    $(systemctl --user is-active robotd)"
echo "    robot-web: $(systemctl --user is-active robot-web)"
python3 robotdctl.py ping || echo "    (ping failed)"
python3 robotdctl.py bal-status | tr ' ' '\n' | grep -E '^(KP|KD|KV|KX|SPIN_WAIT)=' | tr '\n' ' '
echo
REMOTE

echo "==> Done."
