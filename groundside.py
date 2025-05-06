import tkinter as tk
from pymavlink import mavutil
import re
import simplekml
import threading
from decimal import Decimal
import os
import subprocess
import math
from tkintermapview import TkinterMapView

# KML setup
kml = simplekml.Kml()
coordinates = []       # List of (lat, lon)
points = []            # List of KML point objects
map_markers = []       # List of map marker objects
hotspot_count = 1
pattern = re.compile(r"Target:\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)")

# Thread control
stop_flag = threading.Event()
master = None
kml_filename = "targets.kml"

# GUI setup
root = tk.Tk()
root.title("MAVLink Coordinate Receiver")
root.state("zoomed")

frame = tk.Frame(root)
frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

listbox = tk.Listbox(frame, width=40, height=30)
listbox.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

stop_button = tk.Button(frame, text="Stop and Save", command=lambda: stop_program())
stop_button.pack(pady=10)

delete_button = tk.Button(frame, text="Delete Selected", command=lambda: delete_selected())
delete_button.pack(pady=5)

map_widget = TkinterMapView(root, width=500, height=600, corner_radius=0)
map_widget.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
map_widget.set_position(43.4725, -80.5448)
map_widget.set_zoom(16)
map_widget.set_tile_server(
    "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    max_zoom=19
)

# Haversine distance (meters)
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dphi = math.radians(float(lat2) - float(lat1))
    dlambda = math.radians(float(lon2) - float(lon1))
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def update_map_view():
    if len(coordinates) == 1:
        lat, lon = coordinates[0]
        map_widget.set_position(float(lat), float(lon))
        map_widget.set_zoom(18)
    elif len(coordinates) > 1:
        lats = [float(lat) for lat, _ in coordinates]
        lons = [float(lon) for _, lon in coordinates]
        min_lat = min(lats)
        max_lat = max(lats)
        min_lon = min(lons)
        max_lon = max(lons)

        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2
        map_widget.set_position(center_lat, center_lon)

        lat_range = max_lat - min_lat
        lon_range = max_lon - min_lon
        max_range = max(lat_range, lon_range)

        if max_range < 0.0005:
            zoom = 19
        elif max_range < 0.002:
            zoom = 18
        elif max_range < 0.005:
            zoom = 17
        elif max_range < 0.01:
            zoom = 16
        elif max_range < 0.02:
            zoom = 15
        elif max_range < 0.05:
            zoom = 14
        elif max_range < 0.1:
            zoom = 13
        else:
            zoom = 12

        map_widget.set_zoom(zoom)

def delete_selected():
    global hotspot_count
    selected_indices = list(listbox.curselection())[::-1]
    for i in selected_indices:
        listbox.delete(i)
        del coordinates[i]
        del points[i]
        marker = map_markers.pop(i)
        marker.delete()

    # Re-label remaining items
    for idx, (lat, lon) in enumerate(coordinates):
        listbox.delete(idx)
        listbox.insert(idx, f"Hotspot {idx + 1}: {lat}, {lon}")
        points[idx].name = f"Hotspot {idx + 1}"

    hotspot_count = len(coordinates) + 1
    update_map_view()

# Stop and save
def stop_program():
    stop_flag.set()
    if master:
        master.close()
    kml.save(kml_filename)
    print(f"KML saved as {kml_filename}")
    folder_path = os.path.abspath(os.path.dirname(kml_filename))
    subprocess.Popen(f'explorer "{folder_path}"')
    root.quit()

def add_coordinate(lat, lon):
    global hotspot_count
    for i, (existing_lat, existing_lon) in enumerate(coordinates):
        if haversine(lat, lon, existing_lat, existing_lon) < 10:
            print(f"Updating Hotspot {i + 1}: {lat}, {lon}")
            coordinates[i] = (lat, lon)
            points[i].coords = [(float(lon), float(lat))]
            listbox.delete(i)
            listbox.insert(i, f"Hotspot {i + 1}: {lat}, {lon}")
            map_markers[i].set_position(float(lat), float(lon))
            update_map_view()
            return
    label = f"Hotspot {hotspot_count}: {lat}, {lon}"
    listbox.insert(tk.END, label)
    coordinates.append((lat, lon))
    point = kml.newpoint(name=f"Hotspot {hotspot_count}", coords=[(float(lon), float(lat))])
    points.append(point)
    marker = map_widget.set_marker(float(lat), float(lon), text=str(hotspot_count))
    map_markers.append(marker)
    hotspot_count += 1
    update_map_view()

def mavlink_listener():
    global master
    try:
        master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')
        while not stop_flag.is_set():
            try:
                msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
                if msg and msg.get_srcComponent() == 191:
                    text = msg.text
                    match = pattern.match(text)
                    if match:
                        lat = Decimal(match.group(1))
                        lon = Decimal(match.group(2))
                        print(f"Received: {lat}, {lon}")
                        root.after(0, add_coordinate, lat, lon)
            except Exception as e:
                print("Receive error:", e)
    except Exception as e:
        print("Connection failed:", e)

listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
listener_thread.start()
root.mainloop()
