from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('udp:127.0.0.1:14550')

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(10)

while True:
    altitude = vehicle.location.global_relative_frame.alt
    if altitude >= 9.5:  # target altitude - 0.5 meters
        break
    time.sleep(1)

# so apparently, this keeps updating the mission in accordance to the controller.
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0
        self.last_error = 0
        self.last_time = time.time()

    def update(self, measured_value):
        current_time = time.time()
        elapsed_time = current_time - self.last_time

        self.error = self.setpoint - measured_value
        self.error_integral += self.error * elapsed_time
        self.error_derivative = (self.error - self.last_error) / elapsed_time

        output = self.kp * self.error + self.ki * self.error_integral + self.kd * self.error_derivative

        self.last_error = self.error
        self.last_time = current_time

        return output

def control_algorithm(wp):
    pid = PIDController(0.1, 0.05, 0.01, wp.alt)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        output = pid.update(altitude)

        vehicle.simple_goto(LocationGlobalRelative(wp.lat, wp.lon, output))
        time.sleep(1)

        if abs(altitude - wp.alt) <= 0.5:  # target altitude - 0.5 meters
            break

waypoints = [
    LocationGlobalRelative(37.793105, -122.398768, 20),
    LocationGlobalRelative(37.793109, -122.398824, 30),
    LocationGlobalRelative(37.793095, -122.398857, 25),
    LocationGlobalRelative(37.793057, -122.398843, 35),
    LocationGlobalRelative(37.793042, -122.398797, 30),
    LocationGlobalRelative(37.793050, -122.398751, 25),
    LocationGlobalRelative(37.793084, -122.398722, 35),
    LocationGlobalRelative(37.793119, -122.398724, 30)
]

for wp in waypoints:
    control_algorithm(wp)

vehicle.mode = VehicleMode("LAND")

vehicle.close()
