from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Set up connection to vehicle
vehicle = connect('udp:127.0.0.1:14550')

# Set vehicle mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")

# Define base location and target location
base_location = LocationGlobalRelative(37.6189, -122.3750, 10)
target_location = LocationGlobalRelative(37.6200, -122.3770, 20)

# Arm and takeoff
vehicle.armed = True
vehicle.simple_takeoff(base_location.alt)

# Wait for takeoff to complete
while True:
    if abs(vehicle.location.global_relative_frame.alt - base_location.alt) < 1.0:
        print("Reached target altitude")
        break
    time.sleep(1)

# Go to target location
vehicle.simple_goto(target_location)

def distance_to(self, other):
    dlat = other.lat - self.lat
    dlong = other.lon - self.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# Wait for arrival at target location
while True:
    if distance_to(vehicle.location.global_relative_frame, target_location) < 1.0:
        print("Reached target location")
        break
    time.sleep(1)
    
# Set mode to RTL and land
vehicle.mode = VehicleMode("RTL")
while True:
    if vehicle.mode.name == "LAND":
        print("Vehicle landed")
        break
    time.sleep(1)

# Close connection to vehicle
vehicle.close()
