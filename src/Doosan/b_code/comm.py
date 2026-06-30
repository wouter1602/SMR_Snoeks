"""
comm.py — drop-in TCP link for Doosan <-> SMA3.

Same file works on both sides. Pick role with role="server" or "client".

Usage in your main.py:

    from comm import Link

    def on_message(msg):
        print("got:", msg)

    # Doosan side (server):
    link = Link(role="server", host="0.0.0.0",       port=9000, on_message=on_message)
    # SMA3 side (client):
    link = Link(role="client", host="192.168.108.43", port=9000, on_message=on_message)

    link.start()
    link.wait_until_connected(timeout=10)   # block until socket is ready
    link.send("hello")                      # safe to send now

    link.wait_forever()                     # keep main alive while bg thread runs
    # ...or do your own loop and call link.stop() on shutdown
"""

import socket
import threading
import time


class Link:
    def __init__(self, role, host, port, on_message=None):
        assert role in ("server", "client")
        self.role = role
        self.host = host
        self.port = port
        self.on_message = on_message or (lambda msg: None)

        self._sock = None
        self._conn = None
        self._running = False
        self._rx_thread = None
        self._lock = threading.Lock()           # protect concurrent sends
        self._connected_event = threading.Event()

    # ---------- public API ----------
    def start(self):
        self._running = True
        self._rx_thread = threading.Thread(target=self._connect_then_listen, daemon=True)
        self._rx_thread.start()

    def wait_until_connected(self, timeout=None):
        """Block until the socket is connected (or timeout). Returns True if connected."""
        return self._connected_event.wait(timeout=timeout)

    def is_connected(self):
        return self._connected_event.is_set()

    def send(self, msg: str):
        """Send a line of text. Thread-safe."""
        if self._conn is None:
            print("[comm] not connected yet, dropping:", msg)
            return
        data = (msg + "\n").encode()
        with self._lock:
            try:
                self._conn.sendall(data)
            except OSError as e:
                print("[comm] send failed:", e)

    def wait_forever(self):
        """Block the main thread until stop() or Ctrl+C."""
        try:
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("[comm] interrupted, stopping")
            self.stop()

    def stop(self):
        self._running = False
        self._connected_event.clear()
        try:
            if self._conn:
                self._conn.shutdown(socket.SHUT_RDWR)
                self._conn.close()
        except OSError:
            pass
        try:
            if self._sock:
                self._sock.close()
        except OSError:
            pass

    # ---------- internal ----------
    def _connect_then_listen(self):
        if self.role == "server":
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.host, self.port))
            self._sock.listen(1)
            print(f"[comm:server] listening on {self.host}:{self.port}")
            self._conn, addr = self._sock.accept()
            print(f"[comm:server] connected by {addr}")
            self._connected_event.set()
        else:
            while self._running:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    print(f"[comm:client] connecting to {self.host}:{self.port} ...")
                    s.connect((self.host, self.port))
                    self._conn = s
                    print("[comm:client] connected")
                    self._connected_event.set()
                    break
                except OSError as e:
                    print(f"[comm:client] connect failed ({e}); retrying in 2s")
                    time.sleep(2)
            if not self._running:
                return

        # ---- receive loop ----
        buf = b""
        while self._running:
            try:
                chunk = self._conn.recv(1024)
                if not chunk:
                    print("[comm] peer closed connection")
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    msg = line.decode(errors="replace").strip()
                    if msg:
                        try:
                            self.on_message(msg)
                        except Exception as e:
                            print("[comm] on_message error:", e)
            except OSError:
                break

        self._connected_event.clear()
        print("[comm] receive loop exited")
