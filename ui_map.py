import socket
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox

# ==============================
# Config
# ==============================
PORT = 3333
RECV_BUF = 4096

# Heading: 0=N, 1=E, 2=S, 3=W
DIRS = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # dx, dy


class ESPMapUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ESP32 Path Mapper (TCP :3333)")

        # Networking
        self.sock = None
        self.net_thread = None
        self.stop_event = threading.Event()
        self.q = queue.Queue()

        # Map / Pose
        self.reset_map()

        # UI
        self._build_ui()

        # periodic queue polling
        self.root.after(30, self._poll_queue)

    # --------------------------
    # Map logic
    # --------------------------
    def reset_map(self):
        # Robot pose on a grid (cell coords)
        self.x = 0
        self.y = 0
        self.heading = 0  # start facing North
        self.path = [(self.x, self.y)]
        self.events = []  # store received tokens

    def apply_event(self, token: str):
        """
        token: 'R', 'U', 'F' (from ESP32)
        Mapping rule (adjustable):
          R -> heading + 1 (right)
          U -> heading + 2 (u-turn)
          F -> move forward 1 cell (draw)
        """
        token = token.strip().upper()
        if not token:
            return

        # Accept only first char if longer strings arrive
        t = token[0]

        if t not in ("R", "U", "F"):
            # ignore unknown tokens
            return

        self.events.append(t)

        if t == "R":
            self.heading = (self.heading + 1) % 4

        elif t == "U":
            self.heading = (self.heading + 2) % 4

        elif t == "F":
            dx, dy = DIRS[self.heading]
            self.x += dx
            self.y += dy
            self.path.append((self.x, self.y))

        self._redraw()

    # --------------------------
    # UI
    # --------------------------
    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Top controls
        controls = ttk.Frame(frm)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text="ESP32 IP:").grid(row=0, column=0, padx=(0, 6), sticky="w")
        self.ip_var = tk.StringVar(value="172.20.10.2")  # <-- change default
        self.ip_entry = ttk.Entry(controls, textvariable=self.ip_var, width=18)
        self.ip_entry.grid(row=0, column=1, sticky="w")

        self.btn_connect = ttk.Button(controls, text="Connect", command=self.connect)
        self.btn_connect.grid(row=0, column=2, padx=6)

        self.btn_disconnect = ttk.Button(controls, text="Disconnect", command=self.disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=3, padx=6)

        self.btn_reset = ttk.Button(controls, text="Reset Map", command=self.on_reset_map)
        self.btn_reset.grid(row=0, column=4, padx=(12, 0))

        # Status
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(frm, textvariable=self.status_var).grid(row=1, column=0, sticky="w", pady=(8, 0))

        # Canvas + right panel
        body = ttk.Frame(frm)
        body.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        frm.rowconfigure(2, weight=1)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        # Canvas
        self.canvas = tk.Canvas(body, background="white", highlightthickness=1, highlightbackground="#ccc")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # Side panel
        side = ttk.Frame(body, padding=(10, 0, 0, 0))
        side.grid(row=0, column=1, sticky="ns")

        ttk.Label(side, text="Events (latest 200):").grid(row=0, column=0, sticky="w")
        self.events_box = tk.Text(side, width=18, height=22)
        self.events_box.grid(row=1, column=0, sticky="ns")
        self.events_box.configure(state="disabled")

        # Map scale controls
        ttk.Label(side, text="Cell size:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        self.cell_var = tk.IntVar(value=30)
        self.cell_spin = ttk.Spinbox(side, from_=10, to=80, textvariable=self.cell_var, width=6, command=self._redraw)
        self.cell_spin.grid(row=3, column=0, sticky="w")

        ttk.Label(side, text="Grid extent (cells):").grid(row=4, column=0, sticky="w", pady=(10, 0))
        self.extent_var = tk.IntVar(value=20)
        self.extent_spin = ttk.Spinbox(side, from_=10, to=80, textvariable=self.extent_var, width=6, command=self._redraw)
        self.extent_spin.grid(row=5, column=0, sticky="w")

        # Initial draw
        self._redraw()

        # Resize handling
        self.canvas.bind("<Configure>", lambda e: self._redraw())

    def _redraw(self):
        self.canvas.delete("all")

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 2 or h <= 2:
            return

        cell = int(self.cell_var.get())
        extent = int(self.extent_var.get())

        # World (cell coords) -> canvas coords
        cx = w // 2
        cy = h // 2

        def to_canvas(px, py):
            # px,py are cell coords
            return cx + px * cell, cy + py * cell

        # Draw grid
        # extent means from -extent..+extent
        for i in range(-extent, extent + 1):
            x0, y0 = to_canvas(i, -extent)
            x1, y1 = to_canvas(i, extent)
            self.canvas.create_line(x0, y0, x1, y1, fill="#eee")

            x0, y0 = to_canvas(-extent, i)
            x1, y1 = to_canvas(extent, i)
            self.canvas.create_line(x0, y0, x1, y1, fill="#eee")

        # Draw path polyline
        if len(self.path) >= 2:
            pts = []
            for px, py in self.path:
                x, y = to_canvas(px, py)
                pts.extend([x, y])
            self.canvas.create_line(*pts, width=3, fill="black")

        # Draw start point
        sx, sy = to_canvas(self.path[0][0], self.path[0][1])
        self.canvas.create_oval(sx-5, sy-5, sx+5, sy+5, fill="green", outline="")

        # Draw current robot pose
        rx, ry = to_canvas(self.x, self.y)
        self.canvas.create_oval(rx-6, ry-6, rx+6, ry+6, fill="red", outline="")

        # Heading arrow
        dx, dy = DIRS[self.heading]
        ax = rx + dx * cell * 0.6
        ay = ry + dy * cell * 0.6
        self.canvas.create_line(rx, ry, ax, ay, width=3, arrow=tk.LAST, fill="red")

        # Update events box
        self.events_box.configure(state="normal")
        self.events_box.delete("1.0", "end")
        tail = self.events[-200:]
        self.events_box.insert("end", " ".join(tail))
        self.events_box.configure(state="disabled")

    def on_reset_map(self):
        self.reset_map()
        self._redraw()
        self.status_var.set("Map reset. Listening...")

    # --------------------------
    # Networking
    # --------------------------
    def connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showerror("Error", "Please enter ESP32 IP address.")
            return

        self.disconnect()  # ensure clean

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(20.0)
            s.connect((ip, PORT))
            s.settimeout(None)
            self.sock = s
        except Exception as e:
            self.sock = None
            messagebox.showerror("Connect failed", str(e))
            return

        self.stop_event.clear()
        self.net_thread = threading.Thread(target=self._net_loop, daemon=True)
        self.net_thread.start()

        self.btn_connect.configure(state="disabled")
        self.btn_disconnect.configure(state="normal")
        self.status_var.set(f"Connected to {ip}:{PORT} (listening...)")

    def disconnect(self):
        self.stop_event.set()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.status_var.set("Disconnected")

    def _net_loop(self):
        """
        Read lines from socket. Push tokens into queue.
        The ESP32 code typically sends: CONNECTED, then 'U','R','F' lines.
        """
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.sock.recv(RECV_BUF)
                if not data:
                    self.q.put(("status", "Disconnected (socket closed)"))
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        text = line.decode("utf-8", errors="ignore").strip()
                    except Exception:
                        continue
                    if not text:
                        continue
                    # push event
                    self.q.put(("line", text))
            except Exception as e:
                self.q.put(("status", f"Disconnected ({e})"))
                break

        # cleanup
        self.q.put(("done", ""))

    def _poll_queue(self):
        try:
            while True:
                typ, payload = self.q.get_nowait()
                if typ == "line":
                    # If ESP32 sends "CONNECTED" ignore it
                    if payload.upper().startswith("CONNECTED"):
                        continue
                    # apply token
                    self.apply_event(payload)

                elif typ == "status":
                    self.status_var.set(payload)

                elif typ == "done":
                    # ensure UI buttons state
                    self.btn_connect.configure(state="normal")
                    self.btn_disconnect.configure(state="disabled")
        except queue.Empty:
            pass

        self.root.after(30, self._poll_queue)


def main():
    root = tk.Tk()
    app = ESPMapUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()