import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
import math
from pymavlink import mavutil
from serial.tools import list_ports


TELEMETRY_BAUD = 56700


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
    """Generates a boustrophedon path whose waypoints are inside the polygon."""
    footprint = calculate_footprint(alt, fov_deg)
    spacing = footprint * (1.0 - overlap)
    
    min_lat, max_lat = min(poly_lats), max(poly_lats)
    
    R = 6378137.0
    lat_rad = math.radians(min_lat)
    dy_deg = (spacing / R) * (180.0 / math.pi)

    wps = []
    lat = min_lat
    sweep_right = True

    while lat <= max_lat:
        intersections = []
        for i in range(len(poly_lats)):
            next_i = (i + 1) % len(poly_lats)
            lat_1, lon_1 = poly_lats[i], poly_lons[i]
            lat_2, lon_2 = poly_lats[next_i], poly_lons[next_i]
            if (lat_1 <= lat < lat_2) or (lat_2 <= lat < lat_1):
                crossing_lon = lon_1 + (lat - lat_1) * (lon_2 - lon_1) / (lat_2 - lat_1)
                intersections.append(crossing_lon)

        intersections.sort()
        for start in range(0, len(intersections) - 1, 2):
            lane_start = intersections[start]
            lane_end = intersections[start + 1]
            if not sweep_right:
                lane_start, lane_end = lane_end, lane_start
            wps.append((lat, lane_start))
            wps.append((lat, lane_end))

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

def get_home_coordinates(master):
    """Read the stored home position, requesting it when necessary."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    home_position = master.recv_match(
        type="HOME_POSITION", blocking=True, timeout=5
    )
    if home_position and home_position.latitude and home_position.longitude:
        return home_position.latitude / 1e7, home_position.longitude / 1e7

    current_position = master.recv_match(
        type="GLOBAL_POSITION_INT", blocking=True, timeout=5
    )
    if current_position and current_position.lat and current_position.lon:
        print(
            "Warning: flight controller did not publish HOME_POSITION; "
            "using the current GPS position as home."
        )
        return current_position.lat / 1e7, current_position.lon / 1e7

    raise Exception("Could not read the drone's home or GPS position.")

def upload_mission(connection_string, waypoints):
    """Uploads waypoints to the flight controller via MAVLink."""
    master = None
    try:
        connection_string = normalize_connection_string(connection_string)
        print(f"Connecting to {connection_string}...")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat(timeout=5)
        if not master.target_system:
            raise Exception("No heartbeat received from drone.")

        home_lat, home_lon = get_home_coordinates(master)
        print(f"Using home position: {home_lat:.7f}, {home_lon:.7f}")

        if not waypoints:
            raise Exception("The polygon did not produce any valid grid waypoints.")

        mission_items = [
            (
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                home_lat,
                home_lon,
                waypoints[0][2] if waypoints else 0,
            )
        ]
        mission_items.extend(
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt)
            for lat, lon, alt in waypoints
        )
        mission_items.append(
            (mavutil.mavlink.MAV_CMD_NAV_LAND, home_lat, home_lon, 0)
        )
        print(
            "Mission sequence: TAKEOFF home -> "
            f"{len(waypoints)} polygon waypoints -> LAND home"
        )
            
        print("Clearing old mission on the drone...")
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        
        print(
            f"Uploading {len(mission_items)} mission items "
            f"({len(waypoints)} grid waypoints, takeoff, and landing)..."
        )
        master.mav.mission_count_send(
            master.target_system, master.target_component, len(mission_items)
        )
        
        for i, (command, lat, lon, alt) in enumerate(mission_items):
            msg = master.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                blocking=True,
                timeout=5,
            )
            if not msg:
                raise Exception(f"Drone did not request mission item {i}")

            if msg.get_type() == "MISSION_REQUEST_INT":
                master.mav.mission_item_int_send(
                    master.target_system,
                    master.target_component,
                    msg.seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    int(round(lat * 1e7)),
                    int(round(lon * 1e7)),
                    alt,
                )
            else:
                master.mav.mission_item_send(
                    master.target_system,
                    master.target_component,
                    msg.seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    lat,
                    lon,
                    alt,
                )
            
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg and msg.type == 0:
            return True
        else:
            return False
    except PermissionError as e:
        print(
            f"MAVLink Error: cannot open {connection_string}; "
            "the serial port is already in use. Close Mission Planner's "
            "COM5 connection, or configure Mission Planner to forward "
            "MAVLink over UDP and use udp:127.0.0.1:14550 here."
        )
        print(e)
        return False
    except Exception as e:
        print(f"MAVLink Error: {e}")
        return False
    finally:
        if master is not None:
            master.close()

class SearchAreaApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ADDC Search Grid Generator")
        self.root.geometry("450x350")
        
        tk.Label(root, text=f"Telemetry COM port (baud fixed at {TELEMETRY_BAUD}):").pack(pady=5)
        connection_frame = tk.Frame(root)
        connection_frame.pack()
        self.port_combo = ttk.Combobox(
            connection_frame, width=18, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(0, 5))
        tk.Button(
            connection_frame, text="Refresh", command=self.refresh_ports
        ).pack(side=tk.LEFT)
        self.refresh_ports()
        
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

    def refresh_ports(self):
        ports = sorted(
            (port.device for port in list_ports.comports()),
            key=lambda value: (
                0,
                int(value[3:]),
            )
            if value.upper().startswith("COM") and value[3:].isdigit()
            else (1, value),
        )
        self.port_combo["values"] = ports
        if ports:
            if "COM5" in ports:
                self.port_combo.set("COM5")
            else:
                self.port_combo.current(0)
        else:
            self.port_combo.set("")
        
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
        port = self.port_combo.get()
        if not port:
            messagebox.showerror(
                "Error",
                "No COM port selected. Connect the telemetry radio and click Refresh.",
            )
            return
        conn = f"{port},{TELEMETRY_BAUD}"
        
        # Upload
        success = upload_mission(conn, wps)
        if success:
            messagebox.showinfo(
                "Success",
                f"Successfully uploaded {len(wps) + 2} mission items "
                f"({len(wps)} grid waypoints plus takeoff and landing) "
                "to the drone! They should now appear in Mission Planner.",
            )
        else:
            messagebox.showerror(
                "Error",
                f"Failed to upload to {port} at {TELEMETRY_BAUD} baud. "
                "Ensure Mission Planner is disconnected from this COM port.",
            )

if __name__ == "__main__":
    root = tk.Tk()
    app = SearchAreaApp(root)
    root.mainloop()
