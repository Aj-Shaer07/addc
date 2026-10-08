import tkinter as tk
from tkinter import filedialog, messagebox
import math
from pymavlink import mavutil
import time


def normalize_connection_string(connection_string):
    """Normalize common shorthand into pymavlink connection formats."""
    value = connection_string.strip()
    lower_value = value.lower()

    if lower_value.startswith("tcp,"):
        parts = value.split(",", 1)
        if len(parts) == 2 and parts[1].strip().isdigit():
            port = int(parts[1].strip())
            if 0 < port <= 65535:
                return f"tcp:127.0.0.1:{port}"
            raise ValueError(
                "tcp,baud is not valid for SITL. Use tcp:127.0.0.1:5760 or udp:127.0.0.1:14550 instead."
            )
    if lower_value.startswith("udp,"):
        parts = value.split(",", 1)
        if len(parts) == 2 and parts[1].strip().isdigit():
            port = int(parts[1].strip())
            if 0 < port <= 65535:
                return f"udp:127.0.0.1:{port}"
            raise ValueError(
                "udp,baud is not valid for SITL. Use udp:127.0.0.1:14550 or tcp:127.0.0.1:5760 instead."
            )

    return value

def calculate_footprint(alt, fov_deg):
    """Calculates ground footprint width based on altitude and FOV."""
    fov_rad = math.radians(fov_deg)
    width = 2 * alt * math.tan(fov_rad / 2)
    return width

def generate_grid_gps(poly_lats, poly_lons, trees, alt, fov_deg, overlap=0.2):
    """Generates a boustrophedon (lawnmower) path inside a bounding box."""
    footprint = calculate_footprint(alt, fov_deg)
    spacing = footprint * (1.0 - overlap)
    
    # Calculate bounding box
    min_lat, max_lat = min(poly_lats), max(poly_lats)
    min_lon, max_lon = min(poly_lons), max(poly_lons)
    
    R = 6378137.0
    lat_rad = math.radians(min_lat)
    dy_deg = (spacing / R) * (180.0 / math.pi)
    dx_deg = (spacing / (R * math.cos(lat_rad))) * (180.0 / math.pi)
    
    wps = []
    lat = min_lat
    sweep_right = True
    
    while lat <= max_lat:
        lon_start = min_lon if sweep_right else max_lon
        lon_end = max_lon if sweep_right else min_lon
        
        # Start of lane
        wps.append((lat, lon_start))
        # End of lane
        wps.append((lat, lon_end))
        
        lat += dy_deg
        sweep_right = not sweep_right
        
    # Exclude trees (Simple radius exclusion)
    filtered_wps = []
    for w_lat, w_lon in wps:
        valid = True
        for tx, ty, tr in trees:
            # Approximate distance in meters
            dist = math.hypot((w_lat - tx)*111320, (w_lon - ty)*111320*math.cos(math.radians(w_lat)))
            if dist < tr:
                valid = False
        if valid:
            filtered_wps.append((w_lat, w_lon, alt))
            
    return filtered_wps

def upload_mission(connection_string, waypoints):
    """Uploads waypoints to the flight controller via MAVLink."""
    try:
        connection_string = normalize_connection_string(connection_string)
        print(f"Connecting to {connection_string}...")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat(timeout=5)
        if not master.target_system:
            raise Exception("No heartbeat received from drone.")
            
        print("Clearing old mission on the drone...")
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        
        print(f"Uploading {len(waypoints)} grid waypoints...")
        master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
        
        for i, (lat, lon, alt) in enumerate(waypoints):
            msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=3)
            if not msg:
                raise Exception(f"Drone did not request mission item {i}")
                
            master.mav.mission_item_send(
                master.target_system,
                master.target_component,
                msg.seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                lat, lon, alt
            )
            
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg and msg.type == 0:
            return True
        else:
            return False
    except Exception as e:
        print(f"MAVLink Error: {e}")
        return False

class SearchAreaApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ADDC Search Grid Generator")
        self.root.geometry("450x350")
        
        tk.Label(root, text="Telemetry Connection (COM3,57600 | tcp:127.0.0.1:5760 | udp:127.0.0.1:14550):").pack(pady=5)
        self.conn_entry = tk.Entry(root, width=40)
        self.conn_entry.insert(0, "tcp:127.0.0.1:5760")
        self.conn_entry.pack()
        
        self.poly_btn = tk.Button(root, text="Load Mission Planner .poly file", command=self.load_poly)
        self.poly_btn.pack(pady=10)
        
        self.poly_label = tk.Label(root, text="Boundary: None loaded", fg="red")
        self.poly_label.pack()
        
        tk.Label(root, text="Camera FOV (deg):").pack(pady=5)
        self.fov_entry = tk.Entry(root)
        self.fov_entry.insert(0, "55")
        self.fov_entry.pack()
        
        tk.Label(root, text="Search Altitude (m):").pack(pady=5)
        self.alt_entry = tk.Entry(root)
        self.alt_entry.insert(0, "3.0")
        self.alt_entry.pack()
        
        self.upload_btn = tk.Button(root, text="Generate & Upload to Drone", command=self.upload, bg="lightgreen")
        self.upload_btn.pack(pady=20)
        
        self.poly_lats = []
        self.poly_lons = []
        self.trees = []
        
    def load_poly(self):
        filename = filedialog.askopenfilename(filetypes=[("Polygon files", "*.poly"), ("All files", "*.*")])
        if filename:
            try:
                self.poly_lats = []
                self.poly_lons = []
                with open(filename, 'r') as f:
                    for line in f:
                        if line.strip() and not line.startswith('#'):
                            parts = line.strip().split()
                            if len(parts) >= 2:
                                self.poly_lats.append(float(parts[0]))
                                self.poly_lons.append(float(parts[1]))
                self.poly_label.config(text=f"Boundary: {len(self.poly_lats)} points loaded", fg="green")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to parse .poly file: {e}")
                
    def upload(self):
        if not self.poly_lats:
            messagebox.showerror("Error", "Please load a boundary polygon first.")
            return
            
        try:
            alt = float(self.alt_entry.get())
            fov = float(self.fov_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Altitude and FOV must be numeric values.")
            return
        
        # Calculate optimal grid
        wps = generate_grid_gps(self.poly_lats, self.poly_lons, self.trees, alt, fov)
        conn = self.conn_entry.get()
        
        # Upload
        success = upload_mission(conn, wps)
        if success:
            messagebox.showinfo("Success", f"Successfully uploaded {len(wps)} waypoints to the drone! They should now appear in Mission Planner.")
        else:
            messagebox.showerror("Error", "Failed to upload to drone via telemetry. For Mission Planner SITL, use tcp:127.0.0.1:5760 or udp:127.0.0.1:14550.")

if __name__ == "__main__":
    root = tk.Tk()
    app = SearchAreaApp(root)
    root.mainloop()
