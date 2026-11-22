#!/usr/bin/env bash
# Deploy the Raspberry Pi application (the repository's pi/ directory) to the
# robot and restart its services.
#
# Usage, from the repository root on the Mac:
#   tools/deploy-pi.sh                    # robot@r2w-pi.local
#   tools/deploy-pi.sh robot@192.168.1.50 # or another address
#   R2W_PI=robot@other.local tools/deploy-pi.sh
#
# Files are copied into ~/r2w on the Pi. Nothing on the Pi is deleted. The
# script then checks the Python files compile, installs changed systemd user
# service files, restarts robotd and robot-web, and pings the Pico through
# robotd.
set -euo pipefail

TARGET="${1:-${R2W_PI:-robot@r2w-pi.local}}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SOURCE="$ROOT/pi"

if [[ ! -f "$SOURCE/robotd.py" ]]; then
  echo "deploy-pi: cannot find $SOURCE/robotd.py" >&2
  exit 1
fi

echo "==> Copying $SOURCE/ to $TARGET:~/r2w/"
if ssh "$TARGET" 'command -v rsync >/dev/null'; then
  rsync -rtv --exclude '__pycache__' --exclude '*.pyc' --exclude '.DS_Store' \
    "$SOURCE/" "$TARGET:r2w/"
else
  echo "    (rsync not installed on the Pi; using scp)"
  ssh "$TARGET" 'mkdir -p ~/r2w'
  scp -r "$SOURCE"/*.py "$SOURCE"/README.md "$SOURCE"/web "$SOURCE"/systemd \
    "$TARGET:r2w/"
fi

echo "==> Checking, installing services and restarting on the Pi"
ssh "$TARGET" 'bash -s' <<'REMOTE'
set -e
cd ~/r2w
python3 -m py_compile robotctl.py robotd.py robotdctl.py robot_web.py runlog.py
mkdir -p ~/.config/systemd/user
changed=0
for unit in robotd.service robot-web.service; do
  if ! cmp -s "systemd/$unit" "$HOME/.config/systemd/user/$unit"; then
    cp "systemd/$unit" "$HOME/.config/systemd/user/$unit"
    changed=1
  fi
done
if [ "$changed" = 1 ]; then
  systemctl --user daemon-reload
  echo "    service files updated"
fi
systemctl --user restart robotd robot-web
sleep 2
echo "    robotd:    $(systemctl --user is-active robotd)"
echo "    robot-web: $(systemctl --user is-active robot-web)"
python3 robotdctl.py ping || echo "    (ping failed: is the SKR Pico connected?)"
REMOTE

echo "==> Done. Open http://${TARGET#*@}:8080"
