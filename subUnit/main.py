
"""
drone_controller.py

OOP-based drone control using PyMAVLink.
Provides high level functions:

Drone.connect()
Drone.arm()
Drone.takeoff(height)
Drone.set_velocity(vx,vy,vz)
Drone.move_direction(direction,speed)
Drone.goto_position(x,y,z)
Drone.update_position_from_vision(x,y,z)

Designed to integrate with vision based feedback systems.
"""

from pymavlink import mavutil
import time
import math


class Drone:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.master = None

        # Current position from vision system
        self.x = 0
        self.y = 0
        self.z = 0
        self.vel=0
        # Desired target position
        self.target = None

    # ----------------------------------
    # CONNECTION
    # ----------------------------------

    def connect(self):
        print("Connecting to drone...")
        self.master = mavutil.mavlink_connection(self.connection_string)

        self.master.wait_heartbeat()
        print("Connected to system:",
              self.master.target_system,
              self.master.target_component)

    # ----------------------------------
    # MODE
    # ----------------------------------

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    # ----------------------------------
    # ARM / DISARM
    # ----------------------------------

    def arm(self):

        print("Arming motors...")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,0,0,0,0,0
        )

        self.master.motors_armed_wait()
        print("Drone armed")

    def disarm(self):

        print("Disarming")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,0,0,0,0,0
        )

    # ----------------------------------
    # TAKEOFF
    # ----------------------------------

    def takeoff(self, height):

        print("Taking off to", height, "meters")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,0,0,0,
            0,0,
            height
        )

        time.sleep(8)

    # ----------------------------------
    # VELOCITY CONTROL
    # ----------------------------------

    def set_velocity(self, vx, vy, vz):

        """
        velocity in m/s
        NED frame
        """
        self.vel=(vx,vy,vz)
        self.master.mav.set_position_target_local_ned_send(

            0,
            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_FRAME_LOCAL_NED,

            0b0000111111000111,

            0,0,0,

            vx,vy,vz,

            0,0,0,

            0,0
        )

    # ----------------------------------
    # MOVE IN CARDINAL DIRECTION
    # ----------------------------------

    def move_direction(self, direction, speed):

        if direction == "forward":
            self.set_velocity(speed,0,0)
            self.vel=(speed,0,0)

        elif direction == "back":
            self.set_velocity(-speed,0,0)
            self.vel=(-speed,0,0)

        elif direction == "left":
            self.set_velocity(0,-speed,0)
            self.vel=(0,-speed,0)

        elif direction == "right":
            self.set_velocity(0,speed,0)
            self.vel=(0,speed,0)

        elif direction == "up":
            self.set_velocity(0,0,-speed)
            self.vel=(0,0,-speed)   

        elif direction == "down":
            self.set_velocity(0,0,speed)
            self.vel=(0,0,speed)

    # ----------------------------------
    # VISION POSITION UPDATE
    # ----------------------------------

    def update_position_from_vision(self, x, y, z):

        """
        Called from vision system
        """

        self.x = x
        self.y = y
        self.z = z

    # ----------------------------------
    # MOVE TO POSITION USING FEEDBACK
    # ----------------------------------

    def goto_position(self, target_x, target_y, target_z, tolerance=0.1):

        """
        Move drone to target position
        relative to vision frame
        """

        print("Moving to position:", target_x, target_y, target_z)

        while True:

            dx = target_x - self.x
            dy = target_y - self.y
            dz = target_z - self.z

            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist < tolerance:
                print("Target reached")
                self.set_velocity(0,0,0)
                break

            vx = 0.5 * dx
            vy = 0.5 * dy
            vz = 0.5 * dz

            vx = max(min(vx,1),-1)
            vy = max(min(vy,1),-1)
            vz = max(min(vz,1),-1)

            self.set_velocity(vx,vy,vz)

            time.sleep(0.1)

    # ----------------------------------
    # LAND
    # ----------------------------------

    def land(self):

        print("Landing")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,0,0,0,0,0,0
        )


# -------------------------------------------------------
# Example Usage
# -------------------------------------------------------

if __name__ == "__main__":

    drone = Drone()

    drone.connect()

    drone.set_mode("GUIDED")

    drone.arm()

    drone.takeoff(3)

    drone.move_direction("forward",1)

    time.sleep(2)

    drone.set_velocity(0,0,0)

    # Example vision feedback
    drone.update_position_from_vision(0,0,3)

    drone.goto_position(2,2,3)

    drone.land()