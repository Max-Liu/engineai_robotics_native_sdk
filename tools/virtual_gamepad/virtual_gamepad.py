import argparse
import os
import signal
import sys

from core.gamepad_bridge import (
    DEFAULT_CHANNEL,
    DEFAULT_LCM_URL,
    GamepadBridge,
    GamepadBridgeConfig,
    axis_map_from_string,
    button_map_from_string,
    dpad_button_map_from_string,
    diagnose_hid_candidates,
    install_signal_handlers,
    list_gamepads,
    list_hid_candidates,
    monitor_hid_candidates,
    stdin_lcm_relay,
)


DEFAULT_DOCKER_WORKDIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Virtual gamepad UI and physical gamepad LCM bridge."
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Do not start the PyQt UI. This implies --bridge-gamepad.",
    )
    parser.add_argument(
        "--bridge-gamepad",
        action="store_true",
        help="Read a physical gamepad with pygame/SDL and publish virtual gamepad LCM messages.",
    )
    parser.add_argument(
        "--list-gamepads",
        action="store_true",
        help="List physical gamepads visible to pygame/SDL and exit.",
    )
    parser.add_argument(
        "--diagnose-hid",
        action="store_true",
        help="Try to open controller-like HID devices and read one raw report.",
    )
    parser.add_argument(
        "--monitor-hid",
        type=float,
        metavar="SECONDS",
        help="Continuously print changed raw HID reports for the given duration.",
    )
    parser.add_argument(
        "--lcm-url",
        default=DEFAULT_LCM_URL,
        help=f"LCM URL used by the bridge. Default: {DEFAULT_LCM_URL}",
    )
    parser.add_argument(
        "--stdin-lcm-relay",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "pygame", "hid"),
        default="auto",
        help="Physical gamepad backend. Default: auto",
    )
    parser.add_argument(
        "--docker-container",
        help=(
            "Start an LCM relay inside this Docker container using docker exec. "
            "Use this when Docker Desktop blocks host-to-container multicast/UDP."
        ),
    )
    parser.add_argument(
        "--docker-workdir",
        default=DEFAULT_DOCKER_WORKDIR,
        help=f"Repository path inside the Docker container. Default: {DEFAULT_DOCKER_WORKDIR}",
    )
    parser.add_argument(
        "--lcm-channel",
        default=DEFAULT_CHANNEL,
        help=f"LCM channel used by the bridge. Default: {DEFAULT_CHANNEL}",
    )
    parser.add_argument(
        "--gamepad-index",
        type=int,
        default=0,
        help="pygame/SDL gamepad index to read. Default: 0",
    )
    parser.add_argument(
        "--publish-hz",
        type=float,
        default=50.0,
        help="Bridge publish rate in Hz. Default: 50",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.08,
        help="Stick deadzone. Default: 0.08",
    )
    parser.add_argument(
        "--axis-map",
        default="0,1,2,3,4,5",
        metavar="LX,LY,RX,RY,LT,RT",
        help=(
            "Raw pygame axis indexes for left-x,left-y,right-x,right-y,LT,RT. "
            "Default: 0,1,2,3,4,5"
        ),
    )
    parser.add_argument(
        "--button-map",
        default="0,1,2,3,4,5,6,7",
        metavar="A,B,X,Y,LB,RB,BACK,START",
        help=(
            "Raw pygame button indexes for A,B,X,Y,LB,RB,BACK,START. "
            "Use '-' to disable a button. Default: 0,1,2,3,4,5,6,7"
        ),
    )
    parser.add_argument(
        "--dpad-button-map",
        default="-,-,-,-",
        metavar="UP,DOWN,LEFT,RIGHT",
        help=(
            "Optional raw pygame button indexes for D-pad up,down,left,right when "
            "the controller does not expose a hat. Use '-' to disable. Default: -,-,-,-"
        ),
    )
    parser.add_argument(
        "--trigger-mode",
        choices=("auto", "signed", "positive", "raw"),
        default="auto",
        help=(
            "Trigger axis normalization. 'signed' maps -1..1 to 0..1; "
            "'positive' uses max(0, value); 'auto' detects from startup value. "
            "Default: auto"
        ),
    )
    parser.add_argument(
        "--print-raw",
        action="store_true",
        help="Print raw axes/buttons/hats periodically for mapping calibration.",
    )
    return parser


def make_bridge_config(args):
    return GamepadBridgeConfig(
        lcm_url=args.lcm_url,
        backend=args.backend,
        channel=args.lcm_channel,
        docker_container=args.docker_container,
        docker_workdir=args.docker_workdir,
        gamepad_index=args.gamepad_index,
        publish_hz=args.publish_hz,
        deadzone=args.deadzone,
        axis_map=axis_map_from_string(args.axis_map),
        button_map=button_map_from_string(args.button_map),
        dpad_button_map=dpad_button_map_from_string(args.dpad_button_map),
        trigger_mode=args.trigger_mode,
        print_raw=args.print_raw,
    )


def print_gamepads():
    devices = list_gamepads()
    for device in devices:
        print(
            "gamepad[{index}] name={name!r} guid={guid!r} "
            "axes={axes} buttons={buttons} hats={hats}".format(**device)
        )
    if devices:
        return

    print("No gamepads found by pygame/SDL.")
    hid_devices = list_hid_candidates()
    if not hid_devices:
        return

    print("HID devices that look like controllers were found:")
    has_hid_gamepad = False
    for device in hid_devices:
        has_hid_gamepad = has_hid_gamepad or (
            device.get("usage_page") == 1 and device.get("usage") in (4, 5)
        )
        print(
            "hid vendor=0x{vendor_id:04x} product=0x{product_id:04x} "
            "manufacturer={manufacturer!r} product_name={product!r} "
            "usage_page={usage_page} usage={usage} interface={interface_number}".format(
                **device
            )
        )
    if has_hid_gamepad:
        print(
            "A standard HID gamepad was detected. If pygame/SDL does not list it, "
            "try --backend hid. If opening fails, grant Input Monitoring permission "
            "to Terminal/Codex and reconnect the controller."
        )
    else:
        print(
            "If your controller appears only as non-gamepad HID here, switch it to "
            "XInput/DInput gamepad mode or grant Terminal/Codex input-device "
            "permissions, then run --list-gamepads again."
        )


def print_hid_diagnostics():
    diagnostics = diagnose_hid_candidates()
    if not diagnostics:
        print("No controller-like HID devices found.")
        return

    for item in diagnostics:
        prefix = (
            "hid vendor=0x{vendor_id:04x} product=0x{product_id:04x} "
            "manufacturer={manufacturer!r} product_name={product!r} "
            "usage_page={usage_page} usage={usage} interface={interface_number}"
        ).format(**item)
        if item["opened"]:
            print(f"{prefix} opened=True sample={item['sample']}")
        else:
            print(f"{prefix} opened=False error={item['error']}")


def run_hid_monitor(seconds):
    monitor_hid_candidates(duration_seconds=seconds)


def run_headless_bridge(args):
    bridge = GamepadBridge(make_bridge_config(args))
    install_signal_handlers(lambda: setattr(bridge, "_running", False))
    bridge.run_forever()


def run_gui(args):
    from PyQt6.QtCore import QTimer
    from PyQt6.QtGui import QIcon
    from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget, QVBoxLayout, QWidget
    from ui.lcm_manager_widget import LcmManagerWidget
    from ui.virtual_gamepad_widget import VirtualGamepadWidget

    class VirtualGamepadWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Virtual Gamepad")

            self.central_widget = QWidget()
            self.setCentralWidget(self.central_widget)
            self.main_layout = QVBoxLayout(self.central_widget)

            self.init_ui()

        def init_ui(self):
            self.setWindowTitle("Virtual Gamepad")
            current_directory = os.path.dirname(os.path.abspath(__file__))
            self.setWindowIcon(
                QIcon(os.path.join(current_directory, "assets/logo.jpg"))
            )

            self.lcm_manager = LcmManagerWidget()
            self.main_layout.addWidget(self.lcm_manager)

            self.tab_widget = QTabWidget()
            self.tab_widget.setTabPosition(QTabWidget.TabPosition.North)

            self.gamepad_simulator = VirtualGamepadWidget()
            self.tab_widget.addTab(self.gamepad_simulator, "Virtual Gamepad")

            self.main_layout.addWidget(self.tab_widget)

    def signal_handler(_signal, _frame):
        print("Ctrl+C detected! Exiting...")
        QApplication.quit()

    bridge = None
    if args.bridge_gamepad:
        bridge = GamepadBridge(make_bridge_config(args))
        bridge.start()

    signal.signal(signal.SIGINT, signal_handler)

    app = QApplication(sys.argv)
    window = VirtualGamepadWindow()
    window.show()
    if sys.platform != "win32":
        signal.siginterrupt(signal.SIGINT, False)

    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    bridge_timer = None
    if bridge is not None:
        bridge_timer = QTimer()
        bridge_timer.timeout.connect(bridge.publish_once)
        bridge_timer.start(max(1, int(1000.0 / max(args.publish_hz, 1.0))))

    exit_code = app.exec()
    if bridge is not None:
        if bridge_timer is not None:
            bridge_timer.stop()
        bridge.stop()
    return exit_code


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        if args.stdin_lcm_relay:
            stdin_lcm_relay(args.lcm_url, args.lcm_channel)
            return 0

        if args.list_gamepads:
            print_gamepads()
            return 0

        if args.diagnose_hid:
            print_hid_diagnostics()
            return 0

        if args.monitor_hid is not None:
            run_hid_monitor(args.monitor_hid)
            return 0

        if args.no_gui:
            args.bridge_gamepad = True

        if args.bridge_gamepad and args.no_gui:
            run_headless_bridge(args)
            return 0

        return run_gui(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
