"""
Checkpoint Monitor for ESP32 Stage 2 (checkmap)
-------------------------------------------------
Connects to ESP32 via WiFi (TCP port 3333) and shows
which checkpoints (A, B, C) have been found, plus
sensor debug data and current state.

Usage:
    python checkpoint_monitor.py
    python checkpoint_monitor.py 192.168.1.50
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

STATE_NAMES = {
    0: "FOLLOW",
    1: "UTURN",
    2: "STR->RTURN",
    3: "RTURN",
    4: "STRAIGHT_CHECK",
    5: "REVERSE_BACK",
    6: "UTURNTHEN",
}
STATE_COLORS = {
    0: "#2ecc71",  # green
    1: "#e67e22",  # orange
    2: "#3498db",  # blue
    3: "#e74c3c",  # red
    4: "#9b59b6",  # purple
    5: "#e74c3c",  # red
    6: "#f39c12",  # yellow
}

DEBUG_RE = re.compile(
    r"D:(\d+),(\d+),(\d+),(\d+),"
    r"st=(\d+),aL=(\d+),aH=(\d+),cH=(\d+),arm=(\d+),hMs=(\d+)"
    r"(?:,e=([\-\d.]+),c=([\-\d.]+),L=(\d+),R=(\d+))?"
)

CHECKPOINT_RE = re.compile(r"CHECKPOINT_([ABC])")


class CheckpointMonitor:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Checkpoint Monitor (Stage 2)")
        self.root.geometry("660x700")
        self.root.configure(bg="#1a1a2e")

        self.sock = None
        self.net_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()
        self.start_time = None

        self.checkpoints_found = {"A": False, "B": False, "C": False}
        self.checkpoint_times = {"A": None, "B": None, "C": None}
        self.path_map = ""

        self._build_ui()
        self.root.after(50, self._poll_queue)

    def _build_ui(self):
        bg = "#1a1a2e"
        fg = "#ccc"

        frm = tk.Frame(self.root, bg=bg, padx=10, pady=10)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # -- Connection --
        conn = tk.Frame(frm, bg=bg)
        conn.grid(row=0, column=0, sticky="ew")

        tk.Label(conn, text="ESP32 IP:", bg=bg, fg=fg, font=("Consolas", 10)).pack(side="left")
        self.ip_var = tk.StringVar(value="172.20.10.2")
        tk.Entry(conn, textvariable=self.ip_var, width=16, font=("Consolas", 10)).pack(side="left", padx=4)

        self.btn_connect = tk.Button(conn, text="Connect", command=self.connect,
                                     bg="#16213e", fg="#fff", font=("Consolas", 10))
        self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = tk.Button(conn, text="Disconnect", command=self.disconnect,
                                        bg="#16213e", fg="#fff", font=("Consolas", 10), state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Disconnected")
        tk.Label(frm, textvariable=self.status_var, bg=bg, fg="#888",
                 font=("Consolas", 9)).grid(row=1, column=0, sticky="w", pady=(4, 0))

        # -- Checkpoints (big display) --
        cp_frame = tk.LabelFrame(frm, text=" Checkpoints ", bg="#16213e", fg="#aaa",
                                 font=("Consolas", 11, "bold"), padx=15, pady=15)
        cp_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))

        self.cp_labels = {}
        self.cp_status = {}
        cp_colors = {"A": "#2ecc71", "B": "#3498db", "C": "#e74c3c"}

        for i, letter in enumerate(["A", "B", "C"]):
            container = tk.Frame(cp_frame, bg="#16213e")
            container.grid(row=0, column=i, padx=20)

            lbl = tk.Label(container, text=letter, font=("Consolas", 48, "bold"),
                           fg="#333", bg="#16213e")
            lbl.pack()
            self.cp_labels[letter] = (lbl, cp_colors[letter])

            status = tk.Label(container, text="not found", font=("Consolas", 10),
                              fg="#555", bg="#16213e")
            status.pack()
            self.cp_status[letter] = status

        # -- Path map --
        path_frame = tk.Frame(frm, bg=bg)
        path_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        tk.Label(path_frame, text="Path:", bg=bg, fg="#aaa",
                 font=("Consolas", 10)).pack(side="left")
        self.path_label = tk.Label(path_frame, text="-", bg=bg, fg="#fff",
                                   font=("Consolas", 14, "bold"), anchor="w")
        self.path_label.pack(side="left", padx=8)

        # -- State --
        self.state_label = tk.Label(frm, text="---", font=("Consolas", 22, "bold"),
                                    fg="#555", bg="#0f3460", relief="flat", padx=15, pady=6)
        self.state_label.grid(row=4, column=0, sticky="ew", pady=(10, 0))

        # -- Sensors --
        sensor_frame = tk.Frame(frm, bg=bg)
        sensor_frame.grid(row=5, column=0, sticky="ew", pady=(10, 0))

        self.sensor_bars = []
        self.sensor_labels = []
        names = ["S1 (L)", "S2", "S3", "S4 (R)"]
        for i in range(4):
            tk.Label(sensor_frame, text=names[i], font=("Consolas", 10),
                     fg="#aaa", bg=bg, width=7, anchor="e").grid(row=i, column=0, padx=(0, 6))
            canvas = tk.Canvas(sensor_frame, width=280, height=18, bg="#333", highlightthickness=0)
            canvas.grid(row=i, column=1, pady=2)
            self.sensor_bars.append(canvas)
            val_lbl = tk.Label(sensor_frame, text="0", font=("Consolas", 10, "bold"),
                               fg="#fff", bg=bg, width=5, anchor="w")
            val_lbl.grid(row=i, column=2, padx=(6, 0))
            self.sensor_labels.append(val_lbl)

        # -- Flags --
        self.flags_label = tk.Label(frm, text="allLow=- allHigh=- centerHigh=-",
                                    font=("Consolas", 10), fg="#aaa", bg=bg, anchor="w")
        self.flags_label.grid(row=6, column=0, sticky="w", pady=(6, 0))

        self.pid_label = tk.Label(frm, text="error=- corr=- L=- R=-",
                                  font=("Consolas", 10), fg="#aaa", bg=bg, anchor="w")
        self.pid_label.grid(row=7, column=0, sticky="w")

        # -- Log --
        log_frame = tk.Frame(frm, bg=bg)
        log_frame.grid(row=8, column=0, sticky="nsew", pady=(10, 0))
        frm.rowconfigure(8, weight=1)

        self.log_text = tk.Text(log_frame, height=10, font=("Consolas", 9),
                                bg="#0a0a1a", fg="#ccc", insertbackground="#ccc", state="disabled")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.log_text.tag_configure("checkpoint", foreground="#2ecc71", font=("Consolas", 10, "bold"))
        self.log_text.tag_configure("state", foreground="#3498db")
        self.log_text.tag_configure("warn", foreground="#e67e22")
        self.log_text.tag_configure("stop", foreground="#e74c3c", font=("Consolas", 10, "bold"))
        self.log_text.tag_configure("info", foreground="#555")

    def _draw_sensor_bar(self, idx, value):
        canvas = self.sensor_bars[idx]
        canvas.delete("all")
        w = 280
        fill_w = int(value / 1000 * w)
        if value >= 700:
            color = "#2ecc71"
        elif value <= 300:
            color = "#444"
        else:
            color = "#f39c12"
        canvas.create_rectangle(0, 0, fill_w, 18, fill=color, outline="")
        self.sensor_labels[idx].configure(text=str(value), fg=color)

    def _append_log(self, text, tag="info"):
        elapsed = ""
        if self.start_time:
            dt = time.time() - self.start_time
            elapsed = f"[{dt:7.1f}s] "
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{elapsed}{text}\n", tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _mark_checkpoint(self, letter):
        if letter not in self.cp_labels:
            return
        self.checkpoints_found[letter] = True
        lbl, color = self.cp_labels[letter]
        lbl.configure(fg=color)

        elapsed = ""
        if self.start_time:
            t = time.time() - self.start_time
            elapsed = f"{t:.1f}s"
            self.checkpoint_times[letter] = t
        self.cp_status[letter].configure(text=f"FOUND  {elapsed}", fg=color)

    def _handle_line(self, text):
        stripped = text.strip()

        # checkpoint found
        m = CHECKPOINT_RE.search(stripped)
        if m:
            letter = m.group(1)
            self._mark_checkpoint(letter)
            self._append_log(f"CHECKPOINT {letter} FOUND!", "checkpoint")
            return

        # STOP_C
        if "STOP_C" in stripped:
            self._append_log("STOPPED AT C - MISSION COMPLETE", "stop")
            self.state_label.configure(text="STOPPED (C)", fg="#e74c3c")
            return

        # state tokens
        token_map = {
            "CHK": (4, "STRAIGHT_CHECK"),
            "REV": (5, "REVERSE_BACK"),
            "REV_NOCHK": (5, "REVERSE (no chk)"),
            "UTURN_THEN": (6, "UTURNTHEN"),
            "RESUME": (0, "FOLLOW"),
        }
        for token, (st, name) in token_map.items():
            if stripped == token:
                color = STATE_COLORS.get(st, "#ccc")
                self.state_label.configure(text=name, fg=color)
                self._append_log(f"-> {name}", "state")
                return

        # single-char state tokens
        single_map = {"F": 0, "U": 1, "S": 2, "R": 3}
        if stripped in single_map:
            st = single_map[stripped]
            name = STATE_NAMES[st]
            color = STATE_COLORS[st]
            self.state_label.configure(text=name, fg=color)
            self._append_log(f"-> {name}", "state")

            # update path for display
            if stripped in ("R", "U"):
                self.path_map += stripped
                self._update_path_display()
            return

        # debug line D:...
        dm = DEBUG_RE.match(stripped)
        if dm:
            sensors = [int(dm.group(i)) for i in range(1, 5)]
            st = int(dm.group(5))
            aL = int(dm.group(6))
            aH = int(dm.group(7))
            cH = int(dm.group(8))
            arm = int(dm.group(9))
            hMs = int(dm.group(10))

            for i in range(4):
                self._draw_sensor_bar(i, sensors[i])

            name = STATE_NAMES.get(st, f"st={st}")
            color = STATE_COLORS.get(st, "#ccc")
            self.state_label.configure(text=name, fg=color)

            self.flags_label.configure(
                text=f"allLow={'ON' if aL else 'off'}  allHigh={'ON' if aH else 'off'}  "
                     f"centerHigh={'ON' if cH else 'off'}  armed={arm}  holdMs={hMs}")

            if dm.group(11) is not None:
                err = float(dm.group(11))
                corr = float(dm.group(12))
                left = int(dm.group(13))
                right = int(dm.group(14))
                self.pid_label.configure(text=f"error={err:+.3f}  corr={corr:+.2f}  L={left}  R={right}")
            return

        # ignore CONNECTED
        if stripped.upper().startswith("CONNECTED"):
            return

        # other messages
        self._append_log(stripped, "warn")

    def _update_path_display(self):
        found = [l for l in ["A", "B", "C"] if self.checkpoints_found[l]]
        display = self.path_map
        if found:
            display += f"  [{','.join(found)}]"
        self.path_label.configure(text=display if display else "-")

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
        self._append_log(f"Connected to {ip}:{PORT}", "state")

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
                    self._append_log(payload, "warn")
                elif typ == "done":
                    self.btn_connect.configure(state="normal")
                    self.btn_disconnect.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)


def main():
    root = tk.Tk()
    app = CheckpointMonitor(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
