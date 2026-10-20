import customtkinter as ctk
import serial
import serial.tools.list_ports
import time
import threading
import cv2
import csv
import os
import numpy as np
from PIL import Image

# --- CONFIGURATION ---
BAUD_RATE = 115200       
DEFAULT_CAM_ID = 2
HOME_FILE = "robot_home.csv"
WAYPOINT_FILE = "waypoints_color.csv"

# Motor Map
ID_BASE = 11
ID_SHOULDER_L = 12
ID_ELBOW = 14
ID_PITCH = 15
ID_ROLL = 16
ID_GRIPPER = 17
ALL_MOTORS = [ID_BASE, ID_SHOULDER_L, ID_ELBOW, ID_PITCH, ID_ROLL, ID_GRIPPER]
MOTOR_NAMES = {11: "Base", 12: "Shldr", 14: "Elbow", 15: "Pitch", 16: "Roll", 17: "Grip"}
STEPS_PER_DEG = 4096.0 / 360.0

# Scan Zones (X, Y, W, H) - Closer to Center
ZONES = {
    1: (180, 200, 50, 50), 
    2: (270, 200, 50, 50), 
    3: (360, 200, 50, 50)
}

class ColorSorterV4(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("BOSET Color Sorter V4 (Full Feature)")
        self.geometry("1350x900")
        ctk.set_appearance_mode("Dark")

        # System State
        self.ser = None
        self.is_running = False
        self.sliders = {}
        self.waypoints = {} 
        self.lock = threading.Lock()
        
        # Vision State
        self.current_frame_cv = None
        self.picking_mode = None  
        self.color_defs = {1: None, 2: None, 3: None} 
        self.detected_in_zone = {1: "NONE", 2: "NONE", 3: "NONE"}

        # Load Saved Data
        self.load_waypoints_from_csv()

        self.setup_ui()
        self.refresh_ports()
        self.start_camera()
        self.update_gui_image() # Start anti-flicker loop

    def setup_ui(self):
        # === LEFT: VISION ===
        f_left = ctk.CTkFrame(self, width=660)
        f_left.pack(side="left", fill="y", padx=10, pady=10)
        
        ctk.CTkLabel(f_left, text="Interactive Vision", font=("Arial", 18, "bold")).pack(pady=10)
        self.lbl_video = ctk.CTkLabel(f_left, text="Loading Camera...", width=640, height=480, fg_color="black")
        self.lbl_video.pack(pady=5)
        self.lbl_video.bind("<Button-1>", self.on_video_click)
        
        ctk.CTkLabel(f_left, text="1. Click 'Teach Color' -> 2. Click Object in Video").pack(pady=5)
        
        # Color Buttons
        f_colors = ctk.CTkFrame(f_left)
        f_colors.pack(fill="x", pady=10)
        self.btn_c1 = ctk.CTkButton(f_colors, text="Teach Color 1", command=lambda: self.set_pick_mode(1), fg_color="gray")
        self.btn_c1.pack(side="left", padx=10, expand=True)
        self.btn_c2 = ctk.CTkButton(f_colors, text="Teach Color 2", command=lambda: self.set_pick_mode(2), fg_color="gray")
        self.btn_c2.pack(side="left", padx=10, expand=True)
        self.btn_c3 = ctk.CTkButton(f_colors, text="Teach Color 3", command=lambda: self.set_pick_mode(3), fg_color="gray")
        self.btn_c3.pack(side="left", padx=10, expand=True)

        # === RIGHT: CONTROL PANEL ===
        f_right = ctk.CTkScrollableFrame(self, label_text="Control Panel")
        f_right.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        # 1. Connection & Sync
        f_conn = ctk.CTkFrame(f_right)
        f_conn.pack(fill="x", pady=5)
        self.cbox_ports = ctk.CTkComboBox(f_conn, values=["Scanning..."]); self.cbox_ports.pack(side="left", padx=5)
        self.btn_conn = ctk.CTkButton(f_conn, text="CONNECT & SYNC", command=self.connect_serial, fg_color="green", width=120)
        self.btn_conn.pack(side="left", padx=5)

        # 2. Speed & Home
        f_conf = ctk.CTkFrame(f_right)
        f_conf.pack(fill="x", pady=5)
        
        ctk.CTkLabel(f_conf, text="Speed:").pack(side="left", padx=5)
        self.ent_speed = ctk.CTkEntry(f_conf, width=60); self.ent_speed.insert(0, "600")
        self.ent_speed.pack(side="left", padx=5)
        
        ctk.CTkButton(f_conf, text="SET HOME", width=80, fg_color="#555", command=self.set_home).pack(side="right", padx=5)
        ctk.CTkButton(f_conf, text="GO HOME", width=80, fg_color="#1F6AA5", command=self.go_home).pack(side="right", padx=5)

        # 3. Sliders with Labels
        ctk.CTkLabel(f_right, text="Manual Joint Control", font=("Arial", 14, "bold")).pack(pady=10)
        f_sliders = ctk.CTkFrame(f_right)
        f_sliders.pack(fill="x", pady=5)
        
        for mid in ALL_MOTORS:
            r = ctk.CTkFrame(f_sliders, fg_color="transparent")
            r.pack(fill="x", pady=2)
            # Label: "Base (11)"
            name_txt = f"{MOTOR_NAMES[mid]} ({mid})"
            ctk.CTkLabel(r, text=name_txt, width=80, anchor="w").pack(side="left")
            
            s = ctk.CTkSlider(r, from_=0, to=180, command=lambda v, m=mid: self.move_motor(m, v))
            s.set(90)
            s.pack(side="left", fill="x", expand=True, padx=5)
            self.sliders[mid] = s

        # 4. Teaching Matrix (Waypoints)
        ctk.CTkLabel(f_right, text="Teaching Matrix (Saved to CSV)", font=("Arial", 14, "bold")).pack(pady=15)
        
        # Channels
        for i in range(1, 4):
            ctk.CTkButton(f_right, text=f"Set Pick Pose (Channel {i})", 
                          command=lambda x=i: self.save_waypoint_to_csv(f"CHAN_{x}"), 
                          fg_color="#444").pack(fill="x", pady=2, padx=10)
        
        ctk.CTkLabel(f_right, text="--- Bins ---").pack(pady=5)
        
        # Bins
        for i in range(1, 4):
            ctk.CTkButton(f_right, text=f"Set Place Pose (Bin {i})", 
                          command=lambda x=i: self.save_waypoint_to_csv(f"BIN_{x}"), 
                          fg_color="#666").pack(fill="x", pady=2, padx=10)

        # 5. Auto Start
        self.txt_log = ctk.CTkTextbox(f_right, height=100)
        self.txt_log.pack(fill="x", pady=10)
        
        self.btn_auto = ctk.CTkButton(f_right, text="START AUTO SORT", height=50, 
                                      fg_color="gray", state="disabled", command=self.toggle_auto)
        self.btn_auto.pack(fill="x", pady=10)

    # --- CSV LOGIC ---
    def load_waypoints_from_csv(self):
        if not os.path.exists(WAYPOINT_FILE): return
        try:
            with open(WAYPOINT_FILE, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 7: continue
                    name = row[0]
                    angles = [int(x) for x in row[1:]]
                    self.waypoints[name] = angles
            print(f"Loaded {len(self.waypoints)} waypoints.")
        except Exception as e: print(f"CSV Load Error: {e}")

    def save_waypoint_to_csv(self, name):
        # 1. Update Memory
        angles = [int(self.sliders[m].get()) for m in ALL_MOTORS]
        self.waypoints[name] = angles
        
        # 2. Rewrite CSV
        try:
            with open(WAYPOINT_FILE, 'w', newline='') as f:
                writer = csv.writer(f)
                for k, v in self.waypoints.items():
                    writer.writerow([k] + v)
            self.log(f"Saved {name} to CSV.")
        except Exception as e: self.log(f"CSV Save Error: {e}")

    # --- HOME LOGIC ---
    def set_home(self):
        angles = [int(self.sliders[m].get()) for m in ALL_MOTORS]
        with open(HOME_FILE, 'w', newline='') as f:
            csv.writer(f).writerow(angles)
        self.log("Home Position Saved.")

    def go_home(self):
        if not os.path.exists(HOME_FILE):
            self.log("No Home File Found!")
            return
        with open(HOME_FILE, 'r') as f:
            row = next(csv.reader(f))
            angles = [int(x) for x in row]
            self.execute_pose(angles, 2.0)
            self.log("Moved to Home.")

    # --- VISION ---
    def set_pick_mode(self, idx):
        self.picking_mode = idx
        self.log(f"Click video for Color {idx}...")
        # Reset buttons visual
        for btn in [self.btn_c1, self.btn_c2, self.btn_c3]: btn.configure(fg_color="gray")
        # Highlight active
        [self.btn_c1, self.btn_c2, self.btn_c3][idx-1].configure(fg_color="yellow", text_color="black")

    def on_video_click(self, event):
        if self.picking_mode is None or self.current_frame_cv is None: return
        x, y = event.x, event.y
        if 0 <= x < 640 and 0 <= y < 480:
            with self.lock:
                hsv = cv2.cvtColor(self.current_frame_cv, cv2.COLOR_BGR2HSV)
            px = hsv[y, x]
            h, s, v = int(px[0]), int(px[1]), int(px[2])
            
            # Save Range (int32 -> uint8 fix)
            lower = np.array([max(0, h-10), max(50, s-50), max(50, v-50)], dtype="uint8")
            upper = np.array([min(180, h+10), 255, 255], dtype="uint8")
            self.color_defs[self.picking_mode] = (lower, upper)
            
            # Update Button
            with self.lock: rgb = self.current_frame_cv[y, x]
            hex_c = '#%02x%02x%02x' % (rgb[2], rgb[1], rgb[0])
            
            btn = [self.btn_c1, self.btn_c2, self.btn_c3][self.picking_mode-1]
            btn.configure(fg_color=hex_c, text=f"Color {self.picking_mode} Set", text_color="white")
            self.picking_mode = None

    def start_camera(self):
        self.cap = cv2.VideoCapture(DEFAULT_CAM_ID)
        self.cap.set(3, 640); self.cap.set(4, 480)
        threading.Thread(target=self.cam_loop, daemon=True).start()

    def cam_loop(self):
        while True:
            ret, frame = self.cap.read()
            if not ret: time.sleep(0.1); continue
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for z_id, (zx, zy, zw, zh) in ZONES.items():
                cv2.rectangle(frame, (zx, zy), (zx+zw, zy+zh), (0, 255, 255), 2)
                roi = hsv[zy:zy+zh, zx:zx+zw]
                found = "NONE"
                max_a = 0
                for c_id, bnds in self.color_defs.items():
                    if bnds is None: continue
                    mask = cv2.inRange(roi, bnds[0], bnds[1])
                    mask = cv2.erode(mask, None, iterations=1)
                    mask = cv2.dilate(mask, None, iterations=1)
                    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnts:
                        c = max(cnts, key=cv2.contourArea)
                        if cv2.contourArea(c) > 500:
                            found = c_id
                            # Draw visual
                            rx, ry, rw, rh = cv2.boundingRect(c)
                            cv2.rectangle(frame, (zx+rx, zy+ry), (zx+rx+rw, zy+ry+rh), (0,255,0), 2)
                            cv2.putText(frame, f"C{c_id}", (zx+rx, zy+ry), 0, 0.6, (0,255,0), 2)
                self.detected_in_zone[z_id] = found
            
            with self.lock: self.current_frame_cv = frame.copy()
            time.sleep(0.03)

    def update_gui_image(self):
        with self.lock:
            if self.current_frame_cv is not None:
                rgb = cv2.cvtColor(self.current_frame_cv, cv2.COLOR_BGR2RGB)
                img = ctk.CTkImage(Image.fromarray(rgb), size=(640, 480))
                self.lbl_video.configure(image=img, text="")
        self.after(30, self.update_gui_image)

    # --- AUTO SORTING ---
    def toggle_auto(self):
        self.is_running = not self.is_running
        if self.is_running:
            self.btn_auto.configure(text="STOP", fg_color="red")
            threading.Thread(target=self.sorting_loop, daemon=True).start()
        else:
            self.btn_auto.configure(text="START AUTO SORT", fg_color="gray")

    def sorting_loop(self):
        self.log("Auto Started. Going Home...")
        self.go_home()
        time.sleep(1.0)
        
        while self.is_running:
            for i in range(1, 4):
                if not self.is_running: break
                
                did = self.detected_in_zone[i]
                if did != "NONE":
                    c_key, b_key = f"CHAN_{i}", f"BIN_{did}"
                    if c_key in self.waypoints and b_key in self.waypoints:
                        self.log(f"Moving: Ch{i} -> Bin{did}")
                        
                        # Get Poses
                        pk = list(self.waypoints[c_key])
                        pl = list(self.waypoints[b_key])
                        
                        # Sequence
                        # 1. Hover & Open
                        hover = pk.copy(); hover[1] += 20; hover[5] = 0
                        print("Hover Pose")
                        print(hover)
                        # time.sleep(2.5)
                        self.execute_pose(hover, 2.5)
                        
                        
                        # 2. Dive & Grip
                        pk[5] = 0
                        self.execute_pose(pk, 2.0)
                        pk[5] = 75 # Close
                        self.execute_pose(pk, 2.0)
                        print("Pick Pose")
                        print(pk)

                        # 3. Lift
                        hover[5] = 75
                        self.execute_pose(hover, 2.0)
                        print(hover)

                        # 4. Place
                        pl[5] = 75
                        self.execute_pose(pl, 4.0)
                        print(pl)

                        # 5. Drop
                        pl[5] = 0
                        self.execute_pose(pl, 2.5)
                        print(pl)

                        # 6. Home
                        self.go_home()
                        time.sleep(2.5)
            time.sleep(0.1)

    # --- CONNECTION & SYNC ---
    def refresh_ports(self):
        p = [x.device for x in serial.tools.list_ports.comports()]
        self.cbox_ports.configure(values=p if p else ["No Ports"])
        if p: self.cbox_ports.set(p[0])

    def connect_serial(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.cbox_ports.get()
            self.ser.baudrate = BAUD_RATE
            self.ser.dtr = False; self.ser.rts = False
            self.ser.open()
            self.ser.dtr = False; self.ser.rts = False
            time.sleep(2)
            
            # AUTO SYNC: Read motors and update sliders
            self.log("Connected. Syncing Sliders...")
            self.sync_sliders()
            
            self.btn_conn.configure(state="disabled", text="LINKED")
            self.btn_auto.configure(state="normal", fg_color="#D97C23")
        except Exception as e: self.log(f"Err: {e}")

    def sync_sliders(self):
        for mid in ALL_MOTORS:
            raw = self.read_pos(mid)
            if raw is not None:
                deg = int(raw / STEPS_PER_DEG)
                self.sliders[mid].set(deg)
        self.log("Sync Complete.")

    def read_pos(self, mid):
        if not self.ser: return None
        cksm = (~(mid + 4 + 2 + 56 + 2)) & 0xFF
        cmd = bytearray([0xFF, 0xFF, mid, 4, 2, 56, 2, cksm])
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        t = time.time()
        while time.time() - t < 0.1:
            if self.ser.in_waiting >= 8:
                if self.ser.read(1) == b'\xff':
                    pkt = self.ser.read(7)
                    if pkt[1] == mid: return pkt[4] + (pkt[5] << 8)
        return None

    def move_motor(self, mid, val):
        if self.ser and self.ser.is_open:
            self.send_packet(mid, int(val))

    def send_packet(self, mid, angle):
        steps = int(angle * STEPS_PER_DEG)
        steps = max(0, min(4095, steps))
        
        # READ SPEED FROM GUI
        try: spd = int(self.ent_speed.get())
        except: spd = 600
        
        sl, sh = spd & 0xFF, (spd >> 8) & 0xFF
        pl, ph = steps & 0xFF, (steps >> 8) & 0xFF
        cksm = (~(mid + 9 + 3 + 42 + pl + ph + 0 + 0 + sl + sh)) & 0xFF
        msg = bytearray([0xFF, 0xFF, mid, 9, 3, 42, pl, ph, 0, 0, sl, sh, cksm])
        self.ser.write(msg)
        
        if mid == ID_SHOULDER_L:
            inv = 4096 - steps
            pl2, ph2 = inv & 0xFF, (inv >> 8) & 0xFF
            cksm2 = (~(13 + 9 + 3 + 42 + pl2 + ph2 + 0 + 0 + sl + sh)) & 0xFF
            self.ser.write(bytearray([0xFF, 0xFF, 13, 9, 3, 42, pl2, ph2, 0, 0, sl, sh, cksm2]))

    def execute_pose(self, pose, delay):
        # if not self.is_running: return
        for i, mid in enumerate(ALL_MOTORS):
            self.move_motor(mid, pose[i])
            # self.sliders[mid].set(pose[i])
        time.sleep(delay)

    def log(self, msg):
        self.txt_log.insert("end", f"> {msg}\n")
        self.txt_log.see("end")

if __name__ == "__main__":
    app = ColorSorterV4()
    app.mainloop()