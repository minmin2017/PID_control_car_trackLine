"""
Online Debug Monitor for ESP32 Line-Tracking Car
--------------------------------------------------
Connects to ESP32 via WiFi (TCP port 3333) and displays
sensor values, state, flags in real-time.

Usage:
    python serial_monitor.py                # uses default IP
    python serial_monitor.py 192.168.1.50   # specify IP
"""

import sys
import socket
import time
import re
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue

PORT = 3333
RECV_BUF = 4096

STATE_NAMES = {0: "FOLLOW", 1: "UTURN", 2: "STR->RTURN", 3: "RTURN"}
STATE_COLORS = {0: "#2ecc71", 1: "#e67e22", 2: "#3498db", 3: "#e74c3c"}

# Parse: D:n1,n2,n3,n4,st=X,aL=X,aH=X,cH=X,arm=X,hMs=X[,e=X,c=X,L=X,R=X]
DEBUG_RE = re.compile(
    r"D:(\d+),(\d+),(\d+),(\d+),"
    r"st=(\d+),aL=(\d+),aH=(\d+),cH=(\d+),arm=(\d+),hMs=(\d+)"
    r"(?:,e=([\-\d.]+),c=([\-\d.]+),L=(\d+),R=(\d+))?"
)


class DebugMonitor:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Debug Monitor (TCP :3333)")
        self.root.geometry("650x620")
        self.root.configure(bg="#1e1e1e")

        self.sock = None
        self.net_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()
        self.start_time = None
        self.state_count = {0: 0, 1: 0, 2: 0, 3: 0}

        self._build_ui()
        self.root.after(50, self._poll_queue)

    def _build_ui(self):
        style = ttk.Style()
        style.configure("Dark.TFrame", background="#1e1e1e")
        style.configure("Dark.TLabel", background="#1e1e1e", foreground="#ccc")
        style.configure("Dark.TLabelframe", background="#1e1e1e", foreground="#aaa")
        style.configure("Dark.TLabelframe.Label", background="#1e1e1e", foreground="#aaa")

        frm = ttk.Frame(self.root, padding=10, style="Dark.TFrame")
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # -- Connection bar --
        conn = ttk.Frame(frm, style="Dark.TFrame")
        conn.grid(row=0, column=0, sticky="ew")

        ttk.Label(conn, text="ESP32 IP:", style="Dark.TLabel").pack(side="left")
        self.ip_var = tk.StringVar(value="172.20.10.2")
        ip_entry = ttk.Entry(conn, textvariable=self.ip_var, width=16)
        ip_entry.pack(side="left", padx=4)

        self.btn_connect = ttk.Button(conn, text="Connect", command=self.connect)
        self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self.disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)
        ttk.Button(conn, text="Clear Log", command=self.clear_log).pack(side="right")

        # -- Status --
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(frm, textvariable=self.status_var, style="Dark.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 0))

        # -- State display --
        self.state_label = tk.Label(
            frm, text="---", font=("Consolas", 30, "bold"),
            fg="#555", bg="#2a2a2a", relief="flat", padx=20, pady=8
        )
        self.state_label.grid(row=2, column=0, sticky="ew", pady=(10, 0))

        # -- Sensors --
        sensor_frame = tk.Frame(frm, bg="#1e1e1e")
        sensor_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))

        self.sensor_bars = []
        self.sensor_labels = []
        names = ["S1 (L)", "S2", "S3", "S4 (R)"]
        for i in range(4):
            tk.Label(sensor_frame, text=names[i], font=("Consolas", 10),
                     fg="#aaa", bg="#1e1e1e", width=7, anchor="e").grid(row=i, column=0, padx=(0, 6))

            canvas = tk.Canvas(sensor_frame, width=300, height=20, bg="#333", highlightthickness=0)
            canvas.grid(row=i, column=1, pady=2)
            self.sensor_bars.append(canvas)

            val_lbl = tk.Label(sensor_frame, text="0", font=("Consolas", 10, "bold"),
                               fg="#fff", bg="#1e1e1e", width=5, anchor="w")
            val_lbl.grid(row=i, column=2, padx=(6, 0))
            self.sensor_labels.append(val_lbl)

        # -- Flags & PID --
        info_frame = tk.Frame(frm, bg="#1e1e1e")
        info_frame.grid(row=4, column=0, sticky="ew", pady=(10, 0))

        self.flags_label = tk.Label(info_frame, text="allLow=- allHigh=- centerHigh=- armed=- holdMs=-",
                                    font=("Consolas", 10), fg="#aaa", bg="#1e1e1e", anchor="w")
        self.flags_label.pack(fill="x")

        self.pid_label = tk.Label(info_frame, text="error=- corr=- L=- R=-",
                                  font=("Consolas", 10), fg="#aaa", bg="#1e1e1e", anchor="w")
        self.pid_label.pack(fill="x")

        # -- Counters --
        counter_frame = tk.Frame(frm, bg="#1e1e1e")
        counter_frame.grid(row=5, column=0, sticky="ew", pady=(8, 0))
        self.counter_labels = {}
        for i, (st, name) in enumerate(STATE_NAMES.items()):
            color = STATE_COLORS[st]
            lbl = tk.Label(counter_frame, text=f"{name}: 0", font=("Consolas", 10),
                           fg=color, bg="#1e1e1e")
            lbl.grid(row=0, column=i, padx=8)
            self.counter_labels[st] = lbl

        # -- Log --
        log_frame = tk.Frame(frm, bg="#1e1e1e")
        log_frame.grid(row=6, column=0, sticky="nsew", pady=(10, 0))
        frm.rowconfigure(6, weight=1)

        self.log_text = tk.Text(log_frame, height=10, font=("Consolas", 9),
                                bg="#111", fg="#ccc", insertbackground="#ccc", state="disabled")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for st, color in STATE_COLORS.items():
            self.log_text.tag_configure(f"tag_{st}", foreground=color)
        self.log_text.tag_configure("tag_info", foreground="#666")

    def _draw_sensor_bar(self, idx, value):
        canvas = self.sensor_bars[idx]
        canvas.delete("all")
        w = 300
        fill_w = int(value / 1000 * w)

        if value >= 800:
            color = "#2ecc71"  # green = on line
        elif value <= 300:
            color = "#555"     # dim = off line
        else:
            color = "#f39c12"  # orange = mid

        canvas.create_rectangle(0, 0, fill_w, 20, fill=color, outline="")
        self.sensor_labels[idx].configure(text=str(value), fg=color)

    def _update_from_debug(self, m):
        sensors = [int(m.group(i)) for i in range(1, 5)]
        st = int(m.group(5))
        aL = int(m.group(6))
        aH = int(m.group(7))
        cH = int(m.group(8))
        arm = int(m.group(9))
        hMs = int(m.group(10))

        # sensors
        for i in range(4):
            self._draw_sensor_bar(i, sensors[i])

        # state
        name = STATE_NAMES.get(st, "???")
        color = STATE_COLORS.get(st, "#555")
        self.state_label.configure(text=name, fg=color)

        # flags
        self.flags_label.configure(
            text=f"allLow={'ON' if aL else 'off'}  allHigh={'ON' if aH else 'off'}  "
                 f"centerHigh={'ON' if cH else 'off'}  armed={arm}  holdMs={hMs}")

        # PID (only in FOLLOW state - has extra fields)
        if m.group(11) is not None:
            err = float(m.group(11))
            corr = float(m.group(12))
            left = int(m.group(13))
            right = int(m.group(14))
            self.pid_label.configure(
                text=f"error={err:+.3f}  corr={corr:+.2f}  L={left}  R={right}")
        else:
            self.pid_label.configure(text=f"error=-  corr=-  L=-  R=-")

    def _append_log(self, text, tag="tag_info"):
        elapsed = ""
        if self.start_time:
            dt = time.time() - self.start_time
            elapsed = f"[{dt:7.1f}s] "
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{elapsed}{text}\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _handle_line(self, text):
        upper = text.strip().upper()

        # state-change tokens (F/U/R/S)
        if upper in ("F", "U", "R", "S"):
            key_map = {"F": 0, "U": 1, "S": 2, "R": 3}
            st = key_map[upper]
            self.state_count[st] += 1
            for k, lbl in self.counter_labels.items():
                lbl.configure(text=f"{STATE_NAMES[k]}: {self.state_count[k]}")
            name = STATE_NAMES[st]
            color = STATE_COLORS[st]
            self._append_log(f"-> {name}", f"tag_{st}")
            return

        # debug line D:...
        m = DEBUG_RE.match(text)
        if m:
            self._update_from_debug(m)
            return

        # other (CONNECTED, etc.)
        if not upper.startswith("CONNECTED"):
            self._append_log(text)

    # -- Networking --
    def connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showerror("Error", "Enter ESP32 IP")
            return
        self.disconnect()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10.0)
            s.connect((ip, PORT))
            s.settimeout(None)
            self.sock = s
        except Exception as e:
            self.sock = None
            messagebox.showerror("Connect failed", str(e))
            return

        self.stop_event.clear()
        self.start_time = time.time()
        self.net_thread = threading.Thread(target=self._net_loop, daemon=True)
        self.net_thread.start()

        self.btn_connect.configure(state="disabled")
        self.btn_disconnect.configure(state="normal")
        self.status_var.set(f"Connected to {ip}:{PORT}")
        self._append_log(f"Connected to {ip}:{PORT}")

    def disconnect(self):
        self.stop_event.set()
        if self.sock:
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except Exception: pass
            try: self.sock.close()
            except Exception: pass
        self.sock = None
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.status_var.set("Disconnected")

    def _net_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.sock.recv(RECV_BUF)
                if not data:
                    self.q.put(("status", "Disconnected (closed)"))
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if text:
                        self.q.put(("line", text))
            except Exception as e:
                self.q.put(("status", f"Disconnected ({e})"))
                break
        self.q.put(("done", ""))

    def _poll_queue(self):
        try:
            while True:
                typ, payload = self.q.get_nowait()
                if typ == "line":
                    self._handle_line(payload)
                elif typ == "status":
                    self.status_var.set(payload)
                    self._append_log(payload)
                elif typ == "done":
                    self.btn_connect.configure(state="normal")
                    self.btn_disconnect.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)


def main():
    root = tk.Tk()
    app = DebugMonitor(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
