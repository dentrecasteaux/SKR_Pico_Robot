# Backup, Deployment and Recovery

## Source of truth

The Git repository on the development computer is the authoritative project.
The Pi directory `~/r2w` is a deployed copy used at runtime.

Do not make unrecorded long-term changes only on the Pi. Bring useful fixes back
into the Git repository, test them and commit them.

## Small-step source control

The project has been developed with one testable change per commit:

```text
edit -> build -> test -> commit
```

Before starting a step:

```text
git status
git log --oneline -10
```

After testing:

```text
git diff
git add <specific files>
git commit -m "clear description of the completed step"
```

Avoid committing build products, editor state, credentials or unrelated
changes.

## Pico firmware deployment

1. Check out the intended Git commit on the development computer.
2. Build with PlatformIO.
3. Upload to the SKR Pico over USB.
4. Confirm the Pico leaves RP2 Boot and enumerates as USB ACM.
5. Run `PING`, `STATUS`, a low-speed leased motion and `STOP`.
6. Record and commit any configuration change only after the physical test.

The exact PlatformIO upload procedure depends on whether the board is accepting
normal upload or requires a BOOTSEL/UF2 recovery.

## Pi deployment

Copy the contents of the repository’s `pi/` directory to:

```text
/home/robot/r2w/
```

The deployed layout should include:

```text
robotctl.py
robotd.py
robotdctl.py
robot_web.py
web/
systemd/
README.md
```

After updating application files:

```text
systemctl --user restart robotd
systemctl --user restart robot-web
```

Then verify `service-status`, `ping`, `status` and a safe low-speed test.

## Service installation

User service definitions are stored in the repository under `pi/systemd/` and
installed to:

```text
/home/robot/.config/systemd/user/
```

Install or update them with:

```text
mkdir -p ~/.config/systemd/user
cp ~/r2w/systemd/robotd.service ~/.config/systemd/user/
cp ~/r2w/systemd/robot-web.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robotd robot-web
sudo loginctl enable-linger robot
```

## Rebuilding the Pi from a blank SD card

Record these non-secret facts outside the card:

- hostname: `r2w-pi`;
- user: `robot`;
- 64-bit Debian/Raspberry Pi OS family;
- trusted Wi-Fi configuration;
- Pico stable USB identity;
- Git commit deployed;
- required package: `python3-serial`;
- user services and lingering enabled.

Recovery sequence:

1. install a current supported 64-bit Raspberry Pi OS image;
2. create the `robot` account and configure trusted Wi-Fi and SSH;
3. update packages and install `python3-serial`;
4. ensure `robot` belongs to `dialout`;
5. deploy `pi/` to `~/r2w`;
6. install and enable the two user services;
7. enable lingering;
8. connect the Pico and check its stable device identity;
9. run the safe incremental tests;
10. open the local web interface.

Keep Wi-Fi passwords, SSH private keys and other credentials out of this
repository.

## Recovering a known firmware state

Git history provides the safe return point. Prefer creating a new corrective
commit or checking out a known commit for diagnosis. Do not erase local work
with destructive Git commands.

Useful milestones are visible with:

```text
git log --oneline
```

If a new firmware build fails physically, stop motion and power safely, record
the symptoms, then redeploy the last tested commit.

