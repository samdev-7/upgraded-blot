"""Serial streamer worker for the Blot UI.

Mirrors stream.py's line-by-line ok/error protocol with a `?` watchdog when
replies are slow. Lives in its own QThread; talks to the UI through Qt signals.
Pause and cancel are threading.Event flags so they work mid-stream.
"""

from __future__ import annotations

import re
import threading
import time
from typing import Optional

import serial
from PySide6.QtCore import QObject, Signal, Slot


_OK_OR_ERR = re.compile(r"ok\r\n|error:\d+")


class SerialWorker(QObject):
    log = Signal(str)                # one-line message for the log view
    connected = Signal(bool)
    state_changed = Signal(str)      # "Idle" | "Streaming" | "Paused" | "Cancelled" | "Error"
    progress = Signal(int, int)      # (sent, total)
    stream_done = Signal(bool)       # True = success, False = cancel/error

    def __init__(self) -> None:
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._pause = threading.Event()
        self._cancel = threading.Event()

    # ---- control flags (safe to call directly from any thread) ----
    def pause(self) -> None:  self._pause.set()
    def resume(self) -> None: self._pause.clear()
    def cancel(self) -> None: self._cancel.set()

    @property
    def is_connected(self) -> bool:
        return self._ser is not None

    # ---- slots (invoked via queued signals from the main thread) ----

    @Slot(str, int)
    def connect_port(self, port: str, baud: int) -> None:
        if self._ser is not None:
            self.log.emit("Already connected.")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=1)
            time.sleep(0.6)
            self._ser.reset_input_buffer()
            self.log.emit(f"Connected to {port} @ {baud} baud.")
            time.sleep(0.25)
            self._drain_into_log()
            self.connected.emit(True)
        except Exception as exc:
            self.log.emit(f"Connect failed: {exc}")
            self._ser = None
            self.connected.emit(False)

    @Slot()
    def disconnect_port(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.log.emit("Disconnected.")
        self.connected.emit(False)

    @Slot(str)
    def send_line(self, line: str) -> None:
        """One-shot command; waits for ok/error."""
        if self._ser is None:
            self.log.emit("Not connected.")
            return
        self._cancel.clear()
        self._send_and_wait(line)

    @Slot(list)
    def stream(self, lines: list[str]) -> None:
        if self._ser is None:
            self.log.emit("Not connected — can't stream.")
            self.stream_done.emit(False)
            return
        self._pause.clear()
        self._cancel.clear()
        clean = [ln.strip() for ln in lines if ln.strip() and not ln.strip().startswith(";")]
        total = len(clean)
        self.log.emit(f"Streaming {total} lines…")
        self.state_changed.emit("Streaming")
        self.progress.emit(0, total)
        for i, line in enumerate(clean, 1):
            while self._pause.is_set() and not self._cancel.is_set():
                self.state_changed.emit("Paused")
                time.sleep(0.1)
            if self._cancel.is_set():
                self.state_changed.emit("Cancelled")
                self.log.emit("Stream cancelled.")
                self.stream_done.emit(False)
                return
            self.state_changed.emit("Streaming")
            ok = self._send_and_wait(line)
            self.progress.emit(i, total)
            if not ok:
                self.state_changed.emit("Error")
                self.stream_done.emit(False)
                return
        self.state_changed.emit("Idle")
        self.log.emit("Stream complete.")
        self.stream_done.emit(True)

    # ---- internals ----

    def _drain_into_log(self) -> None:
        assert self._ser is not None
        data = self._ser.read(self._ser.in_waiting).decode(errors="replace")
        for ln in data.splitlines():
            if ln.strip():
                self.log.emit(f"< {ln}")

    def _send_and_wait(self, raw: str, timeout: float = 60.0) -> bool:
        line = raw.strip()
        if not line or line.startswith(";"):
            return True
        assert self._ser is not None
        self._ser.write((line + "\n").encode())
        self.log.emit(f"> {line}")
        buf = ""
        start = time.time()
        last_poll = start
        while time.time() - start < timeout:
            if self._cancel.is_set():
                return False
            chunk = self._ser.read(self._ser.in_waiting or 1).decode(errors="replace")
            if chunk:
                buf += chunk
                if _OK_OR_ERR.search(buf):
                    for resp in buf.strip().splitlines():
                        if resp.strip() != "ok":
                            self.log.emit(f"< {resp}")
                    if "error" in buf:
                        return False
                    return True
            now = time.time()
            if now - last_poll > 0.5 and now - start > 1.5:
                self._ser.write(b"?")   # watchdog status nudge, same as stream.py
                last_poll = now
        self.log.emit(f"!! timeout waiting for reply to: {line}")
        return False
