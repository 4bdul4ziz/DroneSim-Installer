from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math

vehicle = connect('udp:127.0.0.1:14550')

# prep for liftoff
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(10)

# let the drone cook
while True:
    altitude = vehicle.location.global_relative_frame.alt
    if altitude >= 9.5:  # target altitude - 0.5 meters
        break
    time.sleep(1)

# points to be crossed during flight
waypoints = [
    LocationGlobalRelative(37.6210, -122.3880, 20),
    LocationGlobalRelative(37.6225, -122.3860, 20),
    LocationGlobalRelative(37.6235, -122.3840, 20),
    LocationGlobalRelative(37.6245, -122.3820, 20),
    LocationGlobalRelative(37.6255, -122.3800, 20),
    LocationGlobalRelative(37.6265, -122.3780, 20),
    LocationGlobalRelative(37.6275, -122.3760, 20),
    LocationGlobalRelative(37.6285, -122.3740, 20)
]

def distance_to(self, other):
    dlat = other.lat - self.lat
    dlong = other.lon - self.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# LETSGOOO
for wp in waypoints:
    vehicle.simple_goto(wp)
    while True:
        if distance_to(vehicle.location.global_relative_frame, wp) <= 1.0:
            print("Reached target location")
            break
        time.sleep(1)

# land
vehicle.mode = VehicleMode("LAND")

vehicle.close()
