"""
Serial helper for RTNode (FIREWALL_MODE) and standard RNode hardware.

The RTNode in FIREWALL_MODE uses its serial port exclusively for ASCII
RNS debug log lines — serial_write() is a compile-time no-op in that
build (see Utilities.h:#ifdef FIREWALL_MODE … return; …).  Therefore
this class handles two distinct serial stream types:

  RNode (standard KISS TNC)
      Emits only binary KISS frames.  All log-line methods return nothing.

  RTNode (FIREWALL_MODE)
      Emits only ASCII log lines terminated by \n.  All KISS-frame
      methods are sent to the device (serial_callback still processes
      the bytes) but no KISS response will ever arrive.

Because FEND (0xC0) is outside the printable ASCII range it serves as
a reliable delimiter between the two stream types.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import serial

# ── KISS framing constants ────────────────────────────────────────────────────
FEND  = 0xC0
FESC  = 0xDB
TFEND = 0xDC
TFESC = 0xDD

# ── Command bytes (mirrors Framing.h) ────────────────────────────────────────
CMD_DATA        = 0x00
CMD_FREQUENCY   = 0x01
CMD_BANDWIDTH   = 0x02
CMD_TXPOWER     = 0x03
CMD_SF          = 0x04
CMD_CR          = 0x05
CMD_RADIO_STATE = 0x06
CMD_RADIO_LOCK  = 0x07
CMD_DETECT      = 0x08
CMD_IMPLICIT    = 0x09
CMD_PROMISC     = 0x0E
CMD_STAT_RX     = 0x21
CMD_STAT_TX     = 0x22
CMD_STAT_RSSI   = 0x23
CMD_STAT_SNR    = 0x24
CMD_STAT_CHTM   = 0x25
CMD_STAT_PHYPRM = 0x26
CMD_BOARD       = 0x47
CMD_PLATFORM    = 0x48
CMD_MCU         = 0x49
CMD_FW_VERSION  = 0x50
CMD_ERROR       = 0x90
CMD_UNKNOWN     = 0xFE

DETECT_REQ  = 0x73
DETECT_RESP = 0x46

RADIO_STATE_OFF = 0x00
RADIO_STATE_ON  = 0x01
RADIO_STATE_ASK = 0xFF

RSSI_OFFSET = 157


@dataclass
class KissFrame:
    cmd: int
    payload: bytes

    def __repr__(self) -> str:
        return f"KissFrame(cmd=0x{self.cmd:02X}, payload={self.payload.hex()})"


@dataclass
class RadioConfig:
    frequency: Optional[int]  = None   # Hz
    bandwidth: Optional[int]  = None   # Hz
    txpower:   Optional[int]  = None   # dBm
    sf:        Optional[int]  = None
    cr:        Optional[int]  = None
    state:     Optional[int]  = None   # RADIO_STATE_OFF / ON

    @property
    def complete(self) -> bool:
        return all(v is not None for v in (
            self.frequency, self.bandwidth, self.sf, self.cr, self.state
        ))

    def __str__(self) -> str:
        parts = []
        if self.frequency is not None:
            parts.append(f"freq={self.frequency/1e6:.3f} MHz")
        if self.bandwidth is not None:
            parts.append(f"BW={self.bandwidth/1e3:.1f} kHz")
        if self.sf is not None:
            parts.append(f"SF={self.sf}")
        if self.cr is not None:
            parts.append(f"CR=4/{self.cr}")
        if self.txpower is not None:
            parts.append(f"TXP={self.txpower} dBm")
        if self.state is not None:
            parts.append("radio=" + ("ON" if self.state == RADIO_STATE_ON else "OFF"))
        return ", ".join(parts) if parts else "(empty)"


class KissSerial:
    """
    Thread-safe wrapper around a serial port that handles both KISS frames
    (from standard RNode) and ASCII log lines (from RTNode FIREWALL_MODE).

    The background reader accumulates bytes until it can classify them:
      • FEND (0xC0) → start/end of a KISS frame
      • \n           → end of an ASCII log line
    Both frame types dispatch to separate callback lists.
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
        )
        self._lock = threading.Lock()
        # KISS frame callbacks
        self._callbacks: list[Callable[[KissFrame], None]] = []
        # ASCII log-line callbacks
        self._log_callbacks: list[Callable[[str], None]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._radio_config = RadioConfig()
        self._received_packets: list[tuple[bytes, int, float]] = []  # (data, rssi_dBm, ts)
        self._log_lines: list[tuple[str, float]] = []  # (line, ts)
        self._last_rssi_raw: Optional[int] = None
        self._last_snr_raw:  Optional[int] = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> "KissSerial":
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "KissSerial":
        return self.start()

    def __exit__(self, *_) -> None:
        self.stop()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def on_frame(self, cb: Callable[[KissFrame], None]) -> None:
        """Register a callback invoked for every received KISS frame."""
        self._callbacks.append(cb)

    def on_log_line(self, cb: Callable[[str], None]) -> None:
        """Register a callback invoked for every ASCII log line."""
        self._log_callbacks.append(cb)

    def _dispatch(self, frame: KissFrame) -> None:
        self._update_state(frame)
        for cb in list(self._callbacks):
            try:
                cb(frame)
            except Exception:
                pass

    def _dispatch_log(self, line: str) -> None:
        ts = time.time()
        with self._lock:
            self._log_lines.append((line, ts))
        for cb in list(self._log_callbacks):
            try:
                cb(line)
            except Exception:
                pass

    def _update_state(self, frame: KissFrame) -> None:
        p = frame.payload
        if frame.cmd == CMD_STAT_RSSI and p:
            self._last_rssi_raw = p[0]
        elif frame.cmd == CMD_STAT_SNR and p:
            self._last_snr_raw = p[0]
        elif frame.cmd == CMD_DATA and p:
            rssi = (self._last_rssi_raw - RSSI_OFFSET) if self._last_rssi_raw is not None else None
            self._received_packets.append((bytes(p), rssi, time.time()))
        elif frame.cmd == CMD_FREQUENCY and len(p) >= 4:
            self._radio_config.frequency = struct.unpack(">I", p[:4])[0]
        elif frame.cmd == CMD_BANDWIDTH and len(p) >= 4:
            self._radio_config.bandwidth = struct.unpack(">I", p[:4])[0]
        elif frame.cmd == CMD_TXPOWER and p:
            self._radio_config.txpower = p[0]
        elif frame.cmd == CMD_SF and p:
            self._radio_config.sf = p[0]
        elif frame.cmd == CMD_CR and p:
            self._radio_config.cr = p[0]
        elif frame.cmd == CMD_RADIO_STATE and p:
            self._radio_config.state = p[0]

    # ── KISS + log-line read loop ─────────────────────────────────────────────

    def _read_loop(self) -> None:
        in_frame = False
        escape   = False
        cmd      = CMD_UNKNOWN
        buf      = bytearray()
        log_buf  = bytearray()   # accumulator for ASCII log lines

        while self._running:
            raw = self._ser.read(256)
            if not raw:
                continue
            for byte in raw:
                if byte == FEND:
                    # Dispatch any pending ASCII log text first
                    if log_buf:
                        line = log_buf.decode("utf-8", errors="replace").rstrip("\r")
                        if line:
                            self._dispatch_log(line)
                        log_buf = bytearray()
                    # Dispatch completed KISS frame
                    if in_frame and cmd != CMD_UNKNOWN and buf:
                        self._dispatch(KissFrame(cmd=cmd, payload=bytes(buf)))
                    # Start new KISS frame
                    in_frame = True
                    escape   = False
                    cmd      = CMD_UNKNOWN
                    buf      = bytearray()
                elif in_frame:
                    # Inside a KISS frame
                    if byte == FESC:
                        escape = True
                    else:
                        if escape:
                            if byte == TFEND:
                                byte = FEND
                            elif byte == TFESC:
                                byte = FESC
                            escape = False
                        if cmd == CMD_UNKNOWN:
                            cmd = byte
                        else:
                            buf.append(byte)
                else:
                    # Not in a KISS frame — accumulate ASCII log line
                    if byte == 0x0A:   # LF  → end of line
                        line = log_buf.decode("utf-8", errors="replace").rstrip("\r")
                        if line:
                            self._dispatch_log(line)
                        log_buf = bytearray()
                    elif byte != 0x0D:  # skip bare CR
                        log_buf.append(byte)

    # ── query helpers ─────────────────────────────────────────────────────────

    def _send_frame(self, cmd: int, payload: bytes = b"") -> None:
        def _esc(data: bytes) -> bytes:
            out = bytearray()
            for b in data:
                if b == FEND:
                    out += bytes([FESC, TFEND])
                elif b == FESC:
                    out += bytes([FESC, TFESC])
                else:
                    out.append(b)
            return bytes(out)

        frame = bytes([FEND, cmd]) + _esc(payload) + bytes([FEND])
        with self._lock:
            self._ser.write(frame)

    def query_detect(self) -> None:
        self._send_frame(CMD_DETECT, bytes([DETECT_REQ]))

    def query_radio_state(self) -> None:
        self._send_frame(CMD_RADIO_STATE, bytes([RADIO_STATE_ASK]))

    def query_frequency(self) -> None:
        self._send_frame(CMD_FREQUENCY, b"\x00\x00\x00\x00")

    def query_bandwidth(self) -> None:
        self._send_frame(CMD_BANDWIDTH, b"\x00\x00\x00\x00")

    def query_txpower(self) -> None:
        self._send_frame(CMD_TXPOWER, bytes([0xFF]))

    def query_sf(self) -> None:
        self._send_frame(CMD_SF, bytes([0xFF]))

    def query_cr(self) -> None:
        self._send_frame(CMD_CR, bytes([0xFF]))

    def set_implicit_length(self, length: int = 0) -> None:
        self._send_frame(CMD_IMPLICIT, bytes([length & 0xFF]))

    def query_all_config(self) -> None:
        self.query_frequency()
        self.query_bandwidth()
        self.query_txpower()
        self.query_sf()
        self.query_cr()
        self.query_radio_state()

    def send_packet(self, data: bytes) -> None:
        self._send_frame(CMD_DATA, data)

    def enable_promisc(self) -> None:
        self._send_frame(CMD_PROMISC, bytes([0x01]))

    # ── accessors ─────────────────────────────────────────────────────────────

    @property
    def radio_config(self) -> RadioConfig:
        return self._radio_config

    @property
    def received_packets(self) -> list[tuple[bytes, int, float]]:
        """Returns copies of all (data, rssi_dBm, timestamp) tuples received."""
        with self._lock:
            return list(self._received_packets)

    @property
    def log_lines(self) -> list[tuple[str, float]]:
        """Returns copies of all (line, timestamp) tuples logged."""
        with self._lock:
            return list(self._log_lines)

    def log_count_since(self, t: float) -> int:
        """Number of log lines received at or after time *t*."""
        with self._lock:
            return sum(1 for _, ts in self._log_lines if ts >= t)

    def wait_for_frame(
        self,
        predicate: Callable[[KissFrame], bool],
        timeout: float = 5.0,
    ) -> Optional[KissFrame]:
        """Block until a frame matching *predicate* arrives or *timeout* expires."""
        result: list[KissFrame] = []
        evt = threading.Event()

        def _cb(frame: KissFrame) -> None:
            if not result and predicate(frame):
                result.append(frame)
                evt.set()

        self._callbacks.append(_cb)
        try:
            evt.wait(timeout=timeout)
            return result[0] if result else None
        finally:
            self._callbacks.remove(_cb)

    def wait_for_packet(self, timeout: float = 10.0) -> Optional[tuple[bytes, int, float]]:
        """Block until a CMD_DATA frame arrives. Returns (data, rssi_dBm, ts) or None."""
        before = len(self._received_packets)

        def _pred(frame: KissFrame) -> bool:
            return frame.cmd == CMD_DATA and len(frame.payload) > 0

        frame = self.wait_for_frame(_pred, timeout=timeout)
        if frame is None:
            return None
        pkts = self._received_packets
        if len(pkts) > before:
            return pkts[-1]
        return None

    def wait_for_config(self, timeout: float = 5.0) -> RadioConfig:
        """Query all config params and wait until at least freq+BW+SF arrive."""
        self.query_all_config()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._radio_config.frequency and self._radio_config.bandwidth and self._radio_config.sf:
                break
            time.sleep(0.05)
        return self._radio_config

    def wait_for_log_line(
        self,
        pattern: str = "",
        timeout: float = 10.0,
    ) -> Optional[str]:
        """
        Block until an ASCII log line containing *pattern* arrives, or until
        *timeout* seconds elapse.  An empty *pattern* matches any line.
        Returns the matched line, or None on timeout.
        """
        result: list[str] = []
        evt = threading.Event()

        def _match(s: str) -> bool:
            return (not pattern) or (pattern.lower() in s.lower())

        def _cb(line: str) -> None:
            if not result and _match(line):
                result.append(line)
                evt.set()

        self._log_callbacks.append(_cb)
        try:
            evt.wait(timeout=timeout)
            return result[0] if result else None
        finally:
            try:
                self._log_callbacks.remove(_cb)
            except ValueError:
                pass
