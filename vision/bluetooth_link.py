import json
import threading
import time

import serial
from serial.tools import list_ports
from minibot_vision import Link, CMD_DISABLE


class BluetoothLink(Link):
    def __init__(self, port=None):
        self.guard = threading.RLock()
        self.seq = 0
        self.sent = 0
        self.last_send = 0.0
        self.ser = None
        self.telemetry = {}
        self.last_rx = 0.0
        self.buffer = b''
        self.message = 'Pair MiniBot-Control in Windows, then connect its outgoing COM port'
        if port:
            self.connect(port)

    @property
    def connected(self):
        return self.ser is not None and time.monotonic() - self.last_rx < 0.7

    @property
    def enabled(self):
        return self.connected and self.telemetry.get('state', 0) != 0

    def connect(self, port):
        with self.guard:
            self.close()
            if port not in {p.device for p in list_ports.comports()}:
                self.message = 'Select an available outgoing Bluetooth COM port'
                return False, self.message
            try:
                self.ser = serial.Serial(port, 115200, timeout=0, write_timeout=0.2)
                self.ser.reset_input_buffer()
                self.message = f'{port} open; waiting for MiniBot-Control status'
                return True, self.message
            except (serial.SerialException, OSError) as exc:
                self.message = f'Could not open {port}: {exc}'
                return False, self.message

    def send(self, *args, **kwargs):
        with self.guard:
            if self.ser is None:
                return
            try:
                super().send(*args, **kwargs)
            except (serial.SerialException, OSError):
                self.close()
                self.message = 'Bluetooth write failed; reconnect after checking the robot'

    def poll(self):
        with self.guard:
            if self.ser is None:
                return
            try:
                self.buffer += self.ser.read(min(self.ser.in_waiting, 4096))
                while b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    try:
                        data = json.loads(line)
                    except (ValueError, UnicodeError):
                        continue
                    if not isinstance(data, dict) or not all(k in data for k in ('state', 'seq', 'battery', 'remaining_ms', 'pose_fresh')):
                        continue
                    if data['state'] not in range(7):
                        continue
                    self.telemetry = data
                    self.last_rx = time.monotonic()
                    self.message = 'Robot connected' if data['pose_fresh'] else 'Robot connected; camera pose missing'
                if len(self.buffer) > 8192:
                    self.buffer = b''
            except (serial.SerialException, OSError):
                self.close()
                self.message = 'Bluetooth disconnected; reconnect to continue'

    def status(self):
        with self.guard:
            return {'connected': self.connected, 'message': self.message if self.connected or not self.last_rx else 'Robot status timed out; reconnect',
                    'battery': self.telemetry.get('battery') if self.connected else None,
                    'port': self.ser.port if self.ser and hasattr(self.ser, 'port') else None,
                    'robot': dict(self.telemetry) if self.connected else {}}

    def close(self):
        with self.guard:
            if self.ser:
                try:
                    super().send((0, 0, 0), False, CMD_DISABLE)
                except (serial.SerialException, OSError):
                    pass
                self.ser.close()
            self.ser = None
            self.telemetry = {}
            self.last_rx = 0.0
            self.buffer = b''
            self.message = 'Bluetooth disconnected'
