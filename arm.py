from pymavlink import mavutil
import time

# ----------------------------
# CONNECT TO FLIGHT CONTROLLER
# ----------------------------
print("Connecting to drone...")

master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()

print("Heartbeat received")
print(f"System: {master.target_system}")
print(f"Component: {master.target_component}")

# ----------------------------
# ARM COMMAND
# ----------------------------
print("Arming motors...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 1 = arm, 0 = disarm
    0, 0, 0, 0, 0, 0
)

# ----------------------------
# CONFIRM ARMING
# ----------------------------
while True:

    msg = master.recv_match(type='HEARTBEAT', blocking=True)

    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("Drone is ARMED")
        break

    else:
        print("Waiting for arming...")
        time.sleep(1)

print("Arming complete")