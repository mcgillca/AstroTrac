# AstroTrac X2 Driver

An X2 plugin for controlling an AstroTrac360 mount from TheSkyX, connecting over
TCP/IP (typically to a mount reachable over Wi-Fi, e.g. via a Raspberry Pi bridge).

## Connecting to the mount

TheSkyX's own serial-device dialog is used to point the driver at the mount, even
though the connection is actually TCP/IP rather than a physical serial port:

![Serial Device Settings dialog](docs/images/serial-device-settings.png)

- **Serial device**: set to `TCP/IP`.
- **TCP/IP host**: the mount's IP address on your network. **Replace the example
  address shown here with your own mount's actual IP** - every network is
  different, and using the wrong address will simply fail to connect.
- **TCP/IP port**: `23` (matches the AstroTrac's own control port).

These settings can only be changed while disconnected - click **Cancel** first if
TheSkyX is currently connected to the mount.

## Driver settings

The driver's own settings dialog (Tools -> AstroTrac360 Setup, or similar,
depending on how TheSkyX exposes it):

![AstroTrac360 Setup dialog](docs/images/astrotrac360-setup.png)

- **Hours tracking past Meridian**: how long the mount is allowed to keep
  tracking past the meridian before the driver stops it. This is a software
  safety limit enforced in the driver's own `raDec()` polling; a padded version
  of the same limit is also sent to the mount firmware (2.35 and later) as a
  last-resort backstop, in case TheSkyX itself hangs or the connection drops
  before the driver's own check can act.
- **Horizon limit (degrees altitude)**: the altitude below which the driver
  stops tracking, enforced the same way - driver-side check first, with a
  padded firmware-side backstop behind it.
- **Pulse Guide Rate**: the guide rate (as a fraction of sidereal) used for
  autoguiding pulses sent via `PulseGuide`/`OpenLoopMove`. `0.1x` is a
  reasonable default for most setups; TheSkyX's own guiding documentation
  covers when a different rate is appropriate.

## Reducing an occasional ~200ms guide-pulse stall (Raspberry Pi bridge setups)

If the mount is reached over Wi-Fi via a Raspberry Pi (or similar Linux bridge),
you may see an occasional command take noticeably longer than normal to
complete - repeatable, isolated events landing consistently around 200-220ms,
distinct from the mount's normal ~2-9ms response time. Driver-side logging and
testing traced this to Linux's TCP `rto_min` (minimum retransmission timeout)
setting, which defaults to 200ms - if a packet to or from the mount is ever
lost, the kernel won't retry sooner than that, regardless of the actual
round-trip time on the link.

Lowering `rto_min` for the route to the mount reduced both how often this
happened (roughly 8-10x fewer occurrences) and how long it lasted when it did
(down from a tight ~207-227ms band to well under 60ms in testing). On the
Raspberry Pi, create a NetworkManager dispatcher script that adds the route
with a lower `rto_min` whenever the relevant network interface comes up:

```bash
sudo tee /etc/NetworkManager/dispatcher.d/99-device-rto <<'EOF'
#!/bin/bash
IFACE="$1"
ACTION="$2"

if [ "$IFACE" = "wlan1" ] && [ "$ACTION" = "up" ]; then
    ip route add 10.39.63.74 dev wlan1 rto_min 20ms 2>/dev/null
fi
EOF
sudo chmod +x /etc/NetworkManager/dispatcher.d/99-device-rto
```

**Before using this, change two things to match your own setup:**

- `10.39.63.74` - replace with your mount's actual IP address (the same one
  entered in TheSkyX's Serial Device Settings above).
- `wlan1` - replace with whichever network interface your Pi actually uses to
  reach the mount. Run `nmcli device status` (or `ip addr`) on the Pi to see
  the interface names available - it may be `wlan0` rather than `wlan1`,
  particularly on a Pi with only one Wi-Fi adapter.

The script only fires on the next `up` event for that interface, so reconnect
the Wi-Fi interface (or reboot the Pi) after installing it. To confirm it took
effect, run `ip route show` on the Pi and check that the route to the mount's
IP shows the custom `rto_min`.

This is a Pi/Linux-side network tuning change, not part of the driver itself -
it doesn't require a new build, and it's safe to try independently of which
driver version is installed.
