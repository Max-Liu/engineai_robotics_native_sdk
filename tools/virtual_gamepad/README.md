# visualization

PyQt6-based visualization and debugging tools for the virtual gamepad workflow. The current toolchain provides:

- LCM connection management
- a virtual gamepad publisher
- a physical gamepad to virtual gamepad LCM bridge
- a lightweight message diagnostic script

## Dependencies

- Python 3.7+
- PyQt6
- lcm
- pygame
- hidapi

To install the required dependencies, run the following command:
```bash
pip3 install -r requirements.txt
```

## Usage

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py
```

Available widgets/components:

1. `LcmManagerWidget`: connect and disconnect from the target LCM URL.
2. `VirtualGamepadWidget`: publish virtual gamepad messages for debugging.
3. `gamepad_bridge.py`: publish a physical gamepad as `virtual_gamepad/gamepad_keys`.

The unified entry point is:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py
```

List physical gamepads visible to pygame/SDL:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --list-gamepads
```

If the controller appears only as a HID candidate and not as a pygame gamepad,
switch it to XInput/DInput gamepad mode and rerun the command.

Check whether controller-like HID devices can be opened and sampled:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --diagnose-hid
```

Continuously print raw HID reports while pressing buttons and moving sticks:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --monitor-hid 10
```

Bridge a macOS-connected gamepad without starting the PyQt GUI:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --lcm-url 'udpm://239.255.76.67:7667?ttl=1'
```

For a Bluetooth DUALSHOCK 4 on macOS, use the HID backend if pygame/SDL does
not list it:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --backend hid --lcm-url 'udpm://239.255.76.67:7667?ttl=1'
```

If the HID backend reports `Failed to open DUALSHOCK 4 HID input device`, grant
Input Monitoring permission to Terminal/Codex in macOS System Settings, then
reconnect the controller.

If the bridge runs in the same Linux/Docker network namespace as the executor,
the default `udpm://239.255.76.67:7667?ttl=0` is usually enough. When publishing
from macOS into Docker Desktop, use `ttl=1` so multicast packets can leave the
host process.

If Docker Desktop blocks host-to-container multicast/UDP, run a relay inside the
container instead:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --backend hid \
  --docker-container engineai_robotics_env
```

Print raw inputs to calibrate a controller-specific mapping:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --print-raw
```

Override mappings if the default Xbox-style SDL layout does not match your
controller:

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui \
  --axis-map 0,1,2,3,4,5 \
  --button-map 0,1,2,3,4,5,6,7 \
  --dpad-button-map -,-,-,-
```

## License

This directory is licensed under **GNU General Public License v3.0 or later** (`GPL-3.0-or-later`).
See `LICENSE` for the full license text.

This subdirectory-specific license applies to the virtual gamepad tool itself. Other parts of the repository remain under the repository root license (`LICENSE.txt`) unless explicitly stated otherwise.
