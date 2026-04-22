import os
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Optional

import lcm
from lcm_msgs.data import GamepadKeys


DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
DEFAULT_CHANNEL = "virtual_gamepad/gamepad_keys"


@dataclass
class AxisMap:
    left_x: int = 0
    left_y: int = 1
    right_x: int = 2
    right_y: int = 3
    lt: int = 4
    rt: int = 5


@dataclass
class ButtonMap:
    a: Optional[int] = 0
    b: Optional[int] = 1
    x: Optional[int] = 2
    y: Optional[int] = 3
    lb: Optional[int] = 4
    rb: Optional[int] = 5
    back: Optional[int] = 6
    start: Optional[int] = 7


@dataclass
class DpadButtonMap:
    up: Optional[int] = None
    down: Optional[int] = None
    left: Optional[int] = None
    right: Optional[int] = None


@dataclass
class GamepadBridgeConfig:
    backend: str = "auto"
    lcm_url: str = DEFAULT_LCM_URL
    channel: str = DEFAULT_CHANNEL
    docker_container: Optional[str] = None
    docker_workdir: Optional[str] = None
    gamepad_index: int = 0
    publish_hz: float = 50.0
    deadzone: float = 0.08
    axis_map: AxisMap = field(default_factory=AxisMap)
    button_map: ButtonMap = field(default_factory=ButtonMap)
    dpad_button_map: DpadButtonMap = field(default_factory=DpadButtonMap)
    trigger_mode: str = "auto"
    print_raw: bool = False


class GamepadBridge:
    def __init__(self, config: GamepadBridgeConfig):
        self.config = config
        self._publisher = None
        self._running = False
        self._backend = None

    def start(self):
        self._running = True
        self._backend = self._open_backend()
        self._publisher = self._open_publisher()
        print(
            "Publishing physical gamepad to LCM "
            f"{self.config.channel} on {self.config.lcm_url}"
        )

    def run_forever(self):
        self.start()
        period = 1.0 / max(self.config.publish_hz, 1.0)
        try:
            while self._running:
                start = time.monotonic()
                self.publish_once()
                elapsed = time.monotonic() - start
                time.sleep(max(0.0, period - elapsed))
        finally:
            self.stop()

    def publish_once(self):
        if self._publisher is None:
            return
        msg = self._backend.read_message()
        msg.timestamp = int(time.time() * 1_000_000)
        self._publisher.publish(msg)

    def _open_publisher(self):
        if self.config.docker_container:
            return DockerExecLcmPublisher(
                container=self.config.docker_container,
                workdir=self.config.docker_workdir,
                lcm_url=self.config.lcm_url,
                channel=self.config.channel,
            )
        return LocalLcmPublisher(self.config.lcm_url, self.config.channel)

    def _open_backend(self):
        errors = []

        if self.config.backend in ("auto", "pygame"):
            backend = PygameGamepadBackend(self.config)
            try:
                backend.start()
                return backend
            except Exception as exc:
                errors.append(f"pygame: {exc}")
                if self.config.backend == "pygame":
                    raise

        if self.config.backend in ("auto", "hid"):
            backend = DualShock4HidBackend(self.config)
            try:
                backend.start()
                return backend
            except Exception as exc:
                errors.append(f"hid: {exc}")
                if self.config.backend == "hid":
                    raise

        raise RuntimeError("No usable gamepad backend found. " + "; ".join(errors))

    def _publish_neutral(self, samples):
        if self._publisher is None:
            return
        msg = GamepadKeys()
        for _ in range(samples):
            msg.timestamp = int(time.time() * 1_000_000)
            self._publisher.publish(msg)
            time.sleep(0.02)

    def stop(self):
        self._running = False
        try:
            self._publish_neutral(samples=3)
        finally:
            if self._publisher:
                self._publisher.close()
                self._publisher = None
            if self._backend:
                self._backend.stop()
                self._backend = None


class LocalLcmPublisher:
    def __init__(self, lcm_url, channel):
        self._channel = channel
        self._lcm_handle = lcm.LCM(lcm_url)

    def publish(self, msg):
        self._lcm_handle.publish(self._channel, msg.encode())

    def close(self):
        pass


class DockerExecLcmPublisher:
    def __init__(self, container, workdir, lcm_url, channel):
        if not workdir:
            raise RuntimeError("--docker-workdir is required when --docker-container is used.")

        command = [
            "docker",
            "exec",
            "-i",
            "-w",
            workdir,
            container,
            "python3",
            "tools/virtual_gamepad/virtual_gamepad.py",
            "--stdin-lcm-relay",
            "--lcm-url",
            lcm_url,
            "--lcm-channel",
            channel,
        ]
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE)
        self._command = command
        print(f"Started docker-exec LCM relay in container {container}.")

    def publish(self, msg):
        if self._process.poll() is not None:
            raise RuntimeError(
                "docker-exec LCM relay exited unexpectedly with code "
                f"{self._process.returncode}."
            )
        payload = msg.encode()
        frame = struct.pack(">I", len(payload)) + payload
        try:
            self._process.stdin.write(frame)
            self._process.stdin.flush()
        except BrokenPipeError as exc:
            raise RuntimeError("docker-exec LCM relay pipe closed.") from exc

    def close(self):
        if self._process.poll() is not None:
            return
        if self._process.stdin:
            self._process.stdin.close()
        try:
            self._process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            self._process.wait(timeout=1.0)


def stdin_lcm_relay(lcm_url, channel):
    lcm_handle = lcm.LCM(lcm_url)
    stream = sys.stdin.buffer

    while True:
        header = _read_exact(stream, 4)
        if not header:
            return
        size = struct.unpack(">I", header)[0]
        if size <= 0 or size > 1024 * 1024:
            raise RuntimeError(f"Invalid relay frame size: {size}")
        payload = _read_exact(stream, size)
        if payload is None:
            return
        lcm_handle.publish(channel, payload)


def _read_exact(stream, size):
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            return None if chunks else b""
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class PygameGamepadBackend:
    def __init__(self, config):
        self.config = config
        self._pygame = None
        self._joystick = None
        self._trigger_baseline = {}
        self._last_raw_print_time = 0.0

    def _init_pygame(self):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError(
                "pygame is required for physical gamepad bridging. "
                "Install it with: pip3 install pygame"
            ) from exc

        pygame.init()
        pygame.joystick.init()
        self._pygame = pygame

    def start(self):
        self._init_pygame()
        self._open_joystick()

    def stop(self):
        if self._pygame:
            self._pygame.quit()

    def read_message(self):
        self._pygame.event.pump()
        msg = self._read_message()
        if self.config.print_raw:
            self._print_raw_state()
        return msg

    def _open_joystick(self):
        count = self._pygame.joystick.get_count()
        if count <= 0:
            raise RuntimeError("No gamepad found by pygame/SDL.")
        if self.config.gamepad_index >= count:
            raise RuntimeError(
                f"Gamepad index {self.config.gamepad_index} is out of range; "
                f"pygame found {count} device(s)."
            )

        joystick = self._pygame.joystick.Joystick(self.config.gamepad_index)
        joystick.init()
        self._joystick = joystick
        self._trigger_baseline = {
            self.config.axis_map.lt: self._safe_axis(self.config.axis_map.lt),
            self.config.axis_map.rt: self._safe_axis(self.config.axis_map.rt),
        }
        print(
            f"Using gamepad[{self.config.gamepad_index}]: {joystick.get_name()} "
            f"({joystick.get_numaxes()} axes, {joystick.get_numbuttons()} buttons, "
            f"{joystick.get_numhats()} hats)"
        )

    def _read_message(self):
        msg = GamepadKeys()

        button_values = [
            self._button(self.config.button_map.lb),
            self._button(self.config.button_map.rb),
            self._button(self.config.button_map.a),
            self._button(self.config.button_map.b),
            self._button(self.config.button_map.x),
            self._button(self.config.button_map.y),
            self._button(self.config.button_map.back),
            self._button(self.config.button_map.start),
        ]
        for index, value in enumerate(button_values):
            msg.digital_states[index] = value

        hat_x, hat_y = self._hat()
        dpad = self.config.dpad_button_map
        msg.digital_states[8] = int(hat_y > 0 or self._button(dpad.up))
        msg.digital_states[9] = int(hat_y < 0 or self._button(dpad.down))
        msg.digital_states[10] = int(hat_x < 0 or self._button(dpad.left))
        msg.digital_states[11] = int(hat_x > 0 or self._button(dpad.right))

        axes = self.config.axis_map
        left_x = self._apply_deadzone(self._safe_axis(axes.left_x))
        left_y = self._apply_deadzone(self._safe_axis(axes.left_y))
        right_x = self._apply_deadzone(self._safe_axis(axes.right_x))
        right_y = self._apply_deadzone(self._safe_axis(axes.right_y))

        msg.analog_states[0] = self._trigger_value(axes.lt)
        msg.analog_states[1] = self._trigger_value(axes.rt)

        # Match the existing virtual UI semantics: forward is analog[2], lateral
        # is analog[3], and the C++ virtual adapter inverts analog[3] and [5].
        msg.analog_states[2] = self._clamp(-left_y)
        msg.analog_states[3] = self._clamp(left_x)
        msg.analog_states[4] = self._clamp(-right_y)
        msg.analog_states[5] = self._clamp(right_x)
        return msg

    def _button(self, index):
        if index is None:
            return 0
        if index < 0 or index >= self._joystick.get_numbuttons():
            return 0
        return int(self._joystick.get_button(index))

    def _hat(self):
        if self._joystick.get_numhats() <= 0:
            return 0, 0
        return self._joystick.get_hat(0)

    def _safe_axis(self, index):
        if index is None or index < 0 or index >= self._joystick.get_numaxes():
            return 0.0
        return float(self._joystick.get_axis(index))

    def _trigger_value(self, axis_index):
        raw = self._safe_axis(axis_index)
        mode = self.config.trigger_mode
        if mode == "auto":
            baseline = self._trigger_baseline.get(axis_index, 0.0)
            mode = "signed" if baseline < -0.5 else "positive"

        if mode == "signed":
            value = (raw + 1.0) * 0.5
        elif mode == "positive":
            value = max(0.0, raw)
        else:
            value = raw
        return self._clamp(value, 0.0, 1.0)

    def _apply_deadzone(self, value):
        if abs(value) < self.config.deadzone:
            return 0.0
        return value

    @staticmethod
    def _clamp(value, low=-1.0, high=1.0):
        return max(low, min(high, value))

    def _publish_neutral(self, samples):
        if self.lcm_handle is None:
            return
        msg = GamepadKeys()
        for _ in range(samples):
            msg.timestamp = int(time.time() * 1_000_000)
            self.lcm_handle.publish(self.config.channel, msg.encode())
            time.sleep(0.02)

    def _print_raw_state(self):
        now = time.monotonic()
        if now - self._last_raw_print_time < 0.5:
            return
        self._last_raw_print_time = now
        axes = [
            round(self._safe_axis(index), 3)
            for index in range(self._joystick.get_numaxes())
        ]
        buttons = [
            self._button(index)
            for index in range(self._joystick.get_numbuttons())
        ]
        hats = [
            self._joystick.get_hat(index)
            for index in range(self._joystick.get_numhats())
        ]
        print(f"raw axes={axes} buttons={buttons} hats={hats}")


class DualShock4HidBackend:
    DS4_PRODUCTS = {0x05C4, 0x09CC}

    def __init__(self, config):
        self.config = config
        self._hid = None
        self._handle = None
        self._device_label = None
        self._latest_report = None
        self._last_raw_print_time = 0.0

    def start(self):
        try:
            import hid
        except ImportError as exc:
            raise RuntimeError(
                "hidapi is required for the HID backend. Install it with: pip3 install hidapi"
            ) from exc

        self._hid = hid
        candidates = self._find_devices()
        if not candidates:
            raise RuntimeError("No DUALSHOCK 4 HID gamepad found.")
        if self.config.gamepad_index >= len(candidates):
            raise RuntimeError(
                f"HID gamepad index {self.config.gamepad_index} is out of range; "
                f"found {len(candidates)} DUALSHOCK 4 device(s)."
            )

        device = candidates[self.config.gamepad_index]
        handle = hid.device()
        try:
            handle.open_path(device["path"])
        except Exception as exc:
            raise RuntimeError(
                "Failed to open DUALSHOCK 4 HID input device. On macOS, grant "
                "Input Monitoring permission to Terminal/Codex and reconnect the controller."
            ) from exc

        handle.set_nonblocking(True)
        self._handle = handle
        self._device_label = (
            f"{device.get('product_string') or 'DUALSHOCK 4'} "
            f"0x{device.get('vendor_id', 0):04x}:0x{device.get('product_id', 0):04x}"
        )
        print(f"Using HID backend: {self._device_label}")

    def stop(self):
        if self._handle:
            self._handle.close()
            self._handle = None

    def read_message(self):
        self._drain_reports()
        msg = self._parse_latest_report()
        if self.config.print_raw:
            self._print_raw_state()
        return msg

    def _find_devices(self):
        devices = []
        for device in self._hid.enumerate():
            vendor_id = device.get("vendor_id") or 0
            product_id = device.get("product_id") or 0
            product = (device.get("product_string") or "").lower()
            usage_page = device.get("usage_page")
            usage = device.get("usage")

            is_ds4 = vendor_id == 0x054C and (
                product_id in self.DS4_PRODUCTS or "dualshock 4" in product
            )
            is_gamepad = usage_page == 1 and usage in (4, 5)
            if is_ds4 and is_gamepad:
                devices.append(device)
        return devices

    def _drain_reports(self):
        while True:
            data = self._handle.read(128)
            if not data:
                return
            self._latest_report = list(data)

    def _parse_latest_report(self):
        msg = GamepadKeys()
        data = self._latest_report
        if not data:
            return msg

        layout = self._detect_layout(data)
        if layout is None:
            return msg

        axis_base, button_base, trigger_base = layout
        left_x = self._axis(data[axis_base])
        left_y = self._axis(data[axis_base + 1])
        right_x = self._axis(data[axis_base + 2])
        right_y = self._axis(data[axis_base + 3])

        button0 = data[button_base]
        button1 = data[button_base + 1]
        dpad = button0 & 0x0F

        square = bool(button0 & 0x10)
        cross = bool(button0 & 0x20)
        circle = bool(button0 & 0x40)
        triangle = bool(button0 & 0x80)

        msg.digital_states[0] = int(bool(button1 & 0x01))  # L1
        msg.digital_states[1] = int(bool(button1 & 0x02))  # R1
        msg.digital_states[2] = int(cross)
        msg.digital_states[3] = int(circle)
        msg.digital_states[4] = int(square)
        msg.digital_states[5] = int(triangle)
        msg.digital_states[6] = int(bool(button1 & 0x10))  # Share
        msg.digital_states[7] = int(bool(button1 & 0x20))  # Options

        up, down, left, right = self._dpad_to_buttons(dpad)
        msg.digital_states[8] = int(up)
        msg.digital_states[9] = int(down)
        msg.digital_states[10] = int(left)
        msg.digital_states[11] = int(right)

        msg.analog_states[0] = self._trigger(data[trigger_base])
        msg.analog_states[1] = self._trigger(data[trigger_base + 1])
        msg.analog_states[2] = self._clamp(-left_y)
        msg.analog_states[3] = self._clamp(left_x)
        msg.analog_states[4] = self._clamp(-right_y)
        msg.analog_states[5] = self._clamp(right_x)
        return msg

    @staticmethod
    def _detect_layout(data):
        if len(data) >= 10 and data[0] == 0x01:
            return 1, 5, 8
        if len(data) >= 12 and data[0] == 0x11:
            return 3, 7, 10
        if len(data) >= 9:
            return 0, 4, 7
        return None

    @staticmethod
    def _axis(value):
        return DualShock4HidBackend._clamp((float(value) - 128.0) / 127.0)

    @staticmethod
    def _trigger(value):
        return DualShock4HidBackend._clamp(float(value) / 255.0, 0.0, 1.0)

    @staticmethod
    def _dpad_to_buttons(value):
        if value > 7:
            return False, False, False, False
        up = value in (0, 1, 7)
        right = value in (1, 2, 3)
        down = value in (3, 4, 5)
        left = value in (5, 6, 7)
        return up, down, left, right

    @staticmethod
    def _clamp(value, low=-1.0, high=1.0):
        return max(low, min(high, value))

    def _print_raw_state(self):
        now = time.monotonic()
        if now - self._last_raw_print_time < 0.5:
            return
        self._last_raw_print_time = now
        if not self._latest_report:
            print("raw hid=<no report yet>")
            return
        hex_bytes = " ".join(f"{byte:02x}" for byte in self._latest_report[:32])
        print(f"raw hid len={len(self._latest_report)} data={hex_bytes}")


def list_gamepads():
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError(
            "pygame is required to list physical gamepads. "
            "Install it with: pip3 install pygame"
        ) from exc

    pygame.init()
    pygame.joystick.init()
    try:
        devices = []
        for index in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            guid = joystick.get_guid() if hasattr(joystick, "get_guid") else ""
            devices.append(
                {
                    "index": index,
                    "name": joystick.get_name(),
                    "guid": guid,
                    "axes": joystick.get_numaxes(),
                    "buttons": joystick.get_numbuttons(),
                    "hats": joystick.get_numhats(),
                }
            )
        return devices
    finally:
        pygame.quit()


def list_hid_candidates():
    try:
        import hid
    except ImportError:
        return []

    keywords = ("8bitdo", "game", "controller", "joystick", "xbox", "dual")
    devices = []
    for device in hid.enumerate():
        name = " ".join(
            str(device.get(key, ""))
            for key in ("manufacturer_string", "product_string")
        ).lower()
        if not any(keyword in name for keyword in keywords):
            continue

        devices.append(
            {
                "vendor_id": device.get("vendor_id") or 0,
                "product_id": device.get("product_id") or 0,
                "manufacturer": device.get("manufacturer_string"),
                "product": device.get("product_string"),
                "usage_page": device.get("usage_page"),
                "usage": device.get("usage"),
                "interface_number": device.get("interface_number"),
            }
        )
    return devices


def diagnose_hid_candidates(read_timeout_ms=500):
    try:
        import hid
    except ImportError as exc:
        raise RuntimeError(
            "hidapi is required for HID diagnostics. Install it with: pip3 install hidapi"
        ) from exc

    diagnostics = []
    for device in hid.enumerate():
        vendor_id = device.get("vendor_id") or 0
        product_id = device.get("product_id") or 0
        manufacturer = device.get("manufacturer_string")
        product = device.get("product_string")
        name = f"{manufacturer or ''} {product or ''}".lower()
        if not any(
            keyword in name
            for keyword in ("8bitdo", "game", "controller", "joystick", "xbox", "dual")
        ):
            continue

        result = {
            "vendor_id": vendor_id,
            "product_id": product_id,
            "manufacturer": manufacturer,
            "product": product,
            "usage_page": device.get("usage_page"),
            "usage": device.get("usage"),
            "interface_number": device.get("interface_number"),
            "opened": False,
            "sample": None,
            "error": None,
        }

        try:
            handle = hid.device()
            handle.open_path(device["path"])
            handle.set_nonblocking(False)
            result["opened"] = True
            sample = handle.read(64, read_timeout_ms)
            if sample:
                result["sample"] = list(sample[:32])
            handle.close()
        except Exception as exc:
            result["error"] = str(exc)

        diagnostics.append(result)
    return diagnostics


def monitor_hid_candidates(duration_seconds=10.0, poll_interval=0.01, read_size=64):
    try:
        import hid
    except ImportError as exc:
        raise RuntimeError(
            "hidapi is required for HID monitoring. Install it with: pip3 install hidapi"
        ) from exc

    handles = []
    for device in hid.enumerate():
        vendor_id = device.get("vendor_id") or 0
        product_id = device.get("product_id") or 0
        manufacturer = device.get("manufacturer_string")
        product = device.get("product_string")
        name = f"{manufacturer or ''} {product or ''}".lower()
        if not any(
            keyword in name
            for keyword in ("8bitdo", "game", "controller", "joystick", "xbox", "dual")
        ):
            continue

        label = (
            f"0x{vendor_id:04x}:0x{product_id:04x} "
            f"usage_page={device.get('usage_page')} usage={device.get('usage')} "
            f"interface={device.get('interface_number')}"
        )
        try:
            handle = hid.device()
            handle.open_path(device["path"])
            handle.set_nonblocking(True)
            handles.append((label, handle))
            print(f"opened {label}")
        except Exception as exc:
            print(f"skipped {label}: {exc}")

    if not handles:
        print("No controller-like HID devices could be opened.")
        return

    print(f"Monitoring HID reports for {duration_seconds:.1f}s. Move sticks and press buttons now.")
    deadline = time.monotonic() + duration_seconds
    last_report = {}
    report_count = 0
    try:
        while time.monotonic() < deadline:
            for label, handle in handles:
                data = handle.read(read_size)
                if not data:
                    continue
                report = tuple(data)
                if last_report.get(label) == report:
                    continue
                last_report[label] = report
                report_count += 1
                hex_bytes = " ".join(f"{byte:02x}" for byte in data)
                print(f"{time.monotonic():.3f} {label} len={len(data)} data={hex_bytes}")
            time.sleep(poll_interval)
        if report_count == 0:
            print(
                "No HID input reports were observed. The opened interface is likely "
                "a vendor/configuration interface, not the controller input stream."
            )
    finally:
        for _label, handle in handles:
            handle.close()


def parse_index_list(value, expected_count, name):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != expected_count:
        raise ValueError(f"{name} must contain {expected_count} comma-separated values.")

    result = []
    for part in parts:
        if part in ("", "-", "none", "None"):
            result.append(None)
        else:
            result.append(int(part))
    return result


def axis_map_from_string(value):
    parsed = parse_index_list(value, 6, "--axis-map")
    if any(item is None for item in parsed):
        raise ValueError("--axis-map values cannot be disabled.")
    return AxisMap(*parsed)


def button_map_from_string(value):
    return ButtonMap(*parse_index_list(value, 8, "--button-map"))


def dpad_button_map_from_string(value):
    return DpadButtonMap(*parse_index_list(value, 4, "--dpad-button-map"))


def install_signal_handlers(stop_callback):
    def handler(signum, _frame):
        stop_callback()
        signal.signal(signum, signal.SIG_DFL)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
