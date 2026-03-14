from dronekit import connect, VehicleMode
import time

# -----------------------------
# CONNECT TO VEHICLE
# -----------------------------
print("Connecting to vehicle...")
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)

# -----------------------------
# ARM AND TAKEOFF FUNCTION
# -----------------------------
def arm_and_takeoff(target_altitude):

    print("Basic checks")

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Switching to GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        time.sleep(1)

    print("Arming motors")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(target_altitude)

    # Wait until altitude reached
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude:.2f} m")

        if altitude >= target_altitude * 0.95:
            print("Target altitude reached")
            break

        time.sleep(1)

# -----------------------------
# MAIN MISSION
# -----------------------------
arm_and_takeoff(1.5)

print("Switching to LOITER")
vehicle.mode = VehicleMode("LOITER")

time.sleep(5)

print("Landing...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print("Descending...")
    time.sleep(1)

print("Landed")

vehicle.close()