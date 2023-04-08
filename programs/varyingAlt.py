from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

# Set up connection to vehicle
vehicle = connect('udp:127.0.0.1:14550')

# Set vehicle mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")

# Define waypoints with varying altitude
waypoints = [
    LocationGlobalRelative(37.6205, -122.3880, 10), # altitude of 10 meters
    LocationGlobalRelative(37.6215, -122.3860, 20), # altitude of 20 meters
    LocationGlobalRelative(37.6210, -122.3840, 30), # altitude of 30 meters
]

# Set up mission
cmds = vehicle.commands
cmds.clear()

# Add waypoints to mission
for i, wp in enumerate(waypoints):
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                  wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

# Upload mission to vehicle
cmds.upload()   

# Arm and takeoff
vehicle.armed = True
vehicle.simple_takeoff(waypoints[0].alt)

# Wait for takeoff to complete
while True:
    if abs(vehicle.location.global_relative_frame.alt - waypoints[0].alt) < 1.0:
        print("Reached target altitude")
        break
    time.sleep(1)

# Set mode to AUTO and start mission
vehicle.mode = VehicleMode("AUTO")
print("Starting mission")
vehicle.commands.next = 0

# Monitor mission execution
while True:
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == len(vehicle.commands):
        print("Mission complete")
        vehicle.mode = VehicleMode("RTL")
        break
    time.sleep(1)

# Wait for vehicle to return to launch point and land
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

# Close connection to vehicle
vehicle.close()
