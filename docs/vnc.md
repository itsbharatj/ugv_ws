# TigerVNC Setup

`setup_vnc.sh` installs a separate XFCE virtual desktop using TigerVNC. It does
not mirror the monitor connected to the robot.

The installer supports Ubuntu 22.04 or newer and Debian 12 or newer using
`apt` and `systemd`. Run it as your normal desktop user:

```bash
cd ~/ugv_ws
./setup_vnc.sh
```

Running it through `sudo` is also supported. The VNC session is still created
for the user who invoked `sudo`, not for `root`.

## Secure Connection

By default, TigerVNC listens only on the robot itself. From another computer,
create an SSH tunnel using the hostname or current IP address printed by the
installer:

```bash
ssh -L 5901:localhost:5901 USER@ROBOT
```

Keep that terminal open and connect the VNC viewer to:

```text
localhost:1
```

This is the recommended configuration because the VNC port is not exposed to
the Wi-Fi network. The installer also installs OpenSSH Server when needed.
If UFW is already active, ensure SSH is permitted with
`sudo ufw allow OpenSSH`.

## Direct LAN Access

Direct access must be explicitly enabled and restricted to a source network:

```bash
sudo ufw allow OpenSSH
sudo ufw enable
./setup_vnc.sh --lan --allow-from 192.168.1.0/24
```

This configures encrypted `TLSVnc` access and adds a UFW rule. LAN mode refuses
to start unless UFW is already active. The installer does not enable UFW
automatically because doing so could affect existing SSH or robot networking
rules.

## Options

Use another display or desktop size:

```bash
./setup_vnc.sh --display 2 --geometry 1280x720
```

Reset the VNC password:

```bash
./setup_vnc.sh --reset-password
```

Check service status:

```bash
./setup_vnc.sh status
```

Disable the managed VNC service:

```bash
./setup_vnc.sh uninstall
```

Uninstall keeps installed packages and `~/.vnc/passwd` to avoid deleting user
data. It removes only this installer's configuration block, display assignment,
service activation, and managed firewall rule.

## Notes

- Display `:1` uses TCP port `5901`; display `:2` uses port `5902`.
- Desktop settings are user-wide. Multiple displays for the same user share the
  same session, geometry, and access mode.
- Existing TigerVNC configuration is preserved outside the managed block.
- Existing crontab entries are preserved. A legacy `@reboot` VNC entry for the
  selected display is backed up and replaced by the packaged systemd service.
- The installer does not purge or replace GDM, GNOME, or another local display
  manager.
