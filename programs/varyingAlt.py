from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

vehicle = connect('udp:127.0.0.1:14550')

vehicle.mode = VehicleMode("GUIDED")

# okay, so san francisco is 37.7749° N, 122.4194° W 
waypoints = [
    LocationGlobalRelative(37.6205, -122.3880, 10), # altitude of 10 meters
    LocationGlobalRelative(37.6215, -122.3860, 20), # altitude of 20 meters
    LocationGlobalRelative(37.6210, -122.3840, 30), # altitude of 30 meters
]
# brother decided to stay under 30, somesort of a failsafe mechanism

cmds = vehicle.commands
cmds.clear()

for i, wp in enumerate(waypoints):
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                  wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
cmds.upload()   

vehicle.armed = True
vehicle.simple_takeoff(waypoints[0].alt)

while True:
    if abs(vehicle.location.global_relative_frame.alt - waypoints[0].alt) < 1.0:
        print("Reached target altitude")
        break
    time.sleep(1)

vehicle.mode = VehicleMode("AUTO")
print("Starting mission")
vehicle.commands.next = 0

while True:
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == len(vehicle.commands):
        print("Mission complete")
        vehicle.mode = VehicleMode("RTL")
        break
    time.sleep(1)

while True:
    if vehicle.mode.name == "RTL":
        print("Vehicle returning to launch point")
        break
    time.sleep(1)

while True:
    if vehicle.mode.name == "LAND":
        print("Vehicle landed")
        break
    time.sleep(1)
vehicle.close()
