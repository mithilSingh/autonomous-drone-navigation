from pymavlink import mavutil
import time

# Connect to flight controller
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Function to change mode
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# Arm drone
print("Arming...")
master.arducopter_arm()
master.motors_armed_wait()
print("Drone armed")

# Set GUIDED mode
set_mode("GUIDED")
time.sleep(2)

# Takeoff
target_alt = 1.5
print("Taking off...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,0,0,0,
    0,0,
    target_alt
)

# Wait until altitude reached
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = msg.relative_alt / 1000.0
    print(f"Altitude: {altitude:.2f} m")

    if altitude >= target_alt * 0.95:
        print("Reached target altitude")
        break

# Move forward with velocity
print("Moving forward")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000111111000111,
    0,0,0,
    0.1,0,0,   # vx, vy, vz (m/s)
    0,0,0,
    0,0
)

time.sleep(2)

# Stop movement
print("Stopping")

master.mav.set_position_target_local_ned_send(
    0,
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000111111000111,
    0,0,0,
    0,0,0,
    0,0,0,
    0,0
)

time.sleep(1)

# Land
print("Landing")

set_mode("LAND")