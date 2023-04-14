from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

vehicle = connect('udp:127.0.0.1:14550')

vehicle.mode = VehicleMode("GUIDED")

ground_altitude = vehicle.location.global_frame.alt # so it somehow pisses itself off if the global frame is used
target_altitude = 20.0
altitude = ground_altitude + target_altitude

cmds = vehicle.commands
# dude refuses to work if commands are not cleared even before actually executing anything reeeee
cmds.clear()

# waypoints
wp1 = vehicle.location.global_frame
wp1.alt = altitude
wp2 = vehicle.location.global_frame
wp2.lat += 0.0001
wp2.lon += 0.0001
wp2.alt = altitude
wp3 = vehicle.location.global_frame
wp3.lat += 0.0001
wp3.lon -= 0.0001
wp3.alt = altitude

# return to launch stuff for MAV, according to some random reddit dude 11 years ago, should always have the links at 0
cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0,
              0, 0, 0)
# adding waypoints
cmds.add(cmd)
cmds.upload()

# arm and yeet
vehicle.armed = True
vehicle.simple_takeoff(target_altitude)

while True:
    if abs(vehicle.location.global_relative_frame.alt - altitude) < 1.0:
        print("Reached target altitude")
        break
    time.sleep(1)

vehicle.mode = VehicleMode("AUTO")
print("Starting mission")
vehicle.commands.next = 0

# monitor it
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

# sayonara
vehicle.close()
