"""Small MQTT 3.1.1 client for the ground-station CLI.

It implements only what the CLI needs: CONNECT, SUBSCRIBE, QoS 0/1 PUBLISH,
PUBACK, PINGREQ/PINGRESP, and reconnect.  The CLI stays dependency-free so it
runs on a bare ground-station Python install.
"""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time


class MQTTError(RuntimeError):
    """Raised on protocol or connection failures."""


def _remaining_length(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value % 128
        value //= 128
        if value:
            byte |= 0x80
        out.append(byte)
        if not value:
            return bytes(out)


def _string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack(">H", len(encoded)) + encoded


class MqttClient:
    def __init__(
        self,
        host: str,
        port: int,
        client_id: str,
        username: str | None = None,
        password: str | None = None,
        keepalive: int = 30,
        on_message=None,
        on_connect=None,
    ):
        self.host = host
        self.port = port
        self.client_id = client_id
        self.username = username
        self.password = password
        self.keepalive = keepalive
        self.on_message = on_message
        self.on_connect = on_connect
        self.sock = None
        self.connected = False
        self._packet_id = 0
        self._last_tx = 0.0
        self._acks: set[int] = set()

    def _next_id(self) -> int:
        self._packet_id = (self._packet_id % 0xFFFF) + 1
        return self._packet_id

    def _send(self, packet: bytes) -> None:
        if self.sock is None:
            raise MQTTError("not connected")
        self.sock.sendall(packet)
        self._last_tx = time.monotonic()

    def _recv_exact(self, size: int) -> bytes:
        if self.sock is None:
            raise MQTTError("not connected")
        data = bytearray()
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise MQTTError("connection closed")
            data.extend(chunk)
        return bytes(data)

    def _read_packet(self) -> tuple[int, bytes]:
        header = self._recv_exact(1)[0]
        value = 0
        scale = 1
        for _ in range(4):
            byte = self._recv_exact(1)[0]
            value += (byte & 0x7F) * scale
            if not byte & 0x80:
                break
            scale *= 128
        else:
            raise MQTTError("malformed remaining length")
        return header, self._recv_exact(value)

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.settimeout(5)

        flags = 0x02
        payload = _string(self.client_id)
        if self.username is not None:
            flags |= 0x80
            payload += _string(self.username)
            if self.password is not None:
                flags |= 0x40
                payload += _string(self.password)
        variable = _string("MQTT") + bytes([4, flags]) + struct.pack(
            ">H", self.keepalive
        )
        body = variable + payload
        self._send(bytes([0x10]) + _remaining_length(len(body)) + body)

        header, response = self._read_packet()
        if header >> 4 != 2 or len(response) < 2 or response[1] != 0:
            raise MQTTError(f"CONNACK rejected: {response!r}")

        self.connected = True
        self._last_tx = time.monotonic()
        self._acks.clear()
        if self.on_connect is not None:
            self.on_connect()

    def subscribe(self, topic: str, qos: int = 0) -> None:
        packet_id = self._next_id()
        body = struct.pack(">H", packet_id) + _string(topic) + bytes([qos & 1])
        self._send(bytes([0x82]) + _remaining_length(len(body)) + body)

    def publish(self, topic: str, payload: bytes, qos: int = 1) -> int:
        body = _string(topic)
        packet_id = 0
        header = 0x30
        if qos:
            packet_id = self._next_id()
            header |= 0x02
            body += struct.pack(">H", packet_id)
        body += payload
        self._send(bytes([header]) + _remaining_length(len(body)) + body)
        return packet_id

    def _handle(self, header: int, body: bytes) -> None:
        packet_type = header >> 4
        if packet_type == 3:
            qos = (header >> 1) & 0x03
            topic_len = struct.unpack_from(">H", body, 0)[0]
            topic = body[2 : 2 + topic_len].decode("utf-8")
            offset = 2 + topic_len
            packet_id = 0
            if qos:
                packet_id = struct.unpack_from(">H", body, offset)[0]
                offset += 2
            payload = body[offset:]
            if qos == 1:
                self._send(b"\x40\x02" + struct.pack(">H", packet_id))
            if self.on_message is not None:
                self.on_message(topic, payload)
        elif packet_type == 4:
            if len(body) >= 2:
                self._acks.add(struct.unpack_from(">H", body, 0)[0])
        elif packet_type == 9:
            if len(body) >= 2 and body[1] == 0x80:
                raise MQTTError("subscription rejected")

    def _keepalive(self) -> None:
        if (
            self.connected
            and self.keepalive > 0
            and (time.monotonic() - self._last_tx) >= self.keepalive / 2
        ):
            self._send(b"\xc0\x00")

    def loop(self, timeout: float = 1.0) -> None:
        if self.sock is None:
            raise MQTTError("not connected")
        self.sock.settimeout(timeout)
        try:
            header, body = self._read_packet()
        except socket.timeout:
            pass
        else:
            self._handle(header, body)
        self._keepalive()

    def wait_for_puback(self, packet_id: int, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.loop(0.2)
            if packet_id in self._acks:
                return True
        return False

    def close(self) -> None:
        if self.sock is not None:
            try:
                if self.connected:
                    self.sock.sendall(b"\xe0\x00")
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected = False

    def run_forever(
        self, topic: str, qos: int = 0, stop: threading.Event | None = None
    ) -> None:
        stop = stop or threading.Event()
        delay = 1.0
        while not stop.is_set():
            try:
                self.connect()
                self.subscribe(topic, qos)
                delay = 1.0
                while not stop.is_set():
                    self.loop(1.0)
            except (OSError, MQTTError) as exc:
                self.close()
                if stop.is_set():
                    break
                print(f"mqtt disconnected: {exc}; retry in {delay:.0f}s",
                      file=sys.stderr)
                stop.wait(delay)
            except KeyboardInterrupt:
                break
        self.close()
