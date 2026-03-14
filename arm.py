from pymavlink import mavutil
import time

# Connect to flight controller
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Arm the drone
print("Arming drone...")
master.arducopter_arm()

master.motors_armed_wait()
print("Drone armed")

# Wait for 5 seconds
time.sleep(5)

# Disarm drone
print("Disarming drone...")
master.arducopter_disarm()

master.motors_disarmed_wait()
print("Drone disarmed")
