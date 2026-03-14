from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()

print("Heartbeat received!")