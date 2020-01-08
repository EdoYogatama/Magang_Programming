from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse as ap
import time
import waypoints as wp
import future
import math
from pymavlink import mavutil

# Argument Parser, command to connect to the vehicle
parser  = ap.ArgumentParser(description='Control Copter and send commend in GUIDED mode')
parser.add_argument('--connect', 
                    help='Vehicle connection target string. If not specified, SITL automatically started and used.')
args    = parser.parse_args()

connection_string   = args.connect
sitl                = None

# If connection_string not specified import sitl and start using sitl
if not connection_string:
    import dronekit_sitl as sim
    sitl                = sim.start_default()
    connection_string   = sitl.connection_string()

# Connection to vehicle with dronekit connect
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# define arm and takeoff from dronekit API
def arm_and_tekoff(altitude_target):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
   
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(altitude_target) # Take off to target altitude

    # Wait until the vehicle reaches a safe height
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Going up ",current_altitude)
        if current_altitude >= altitude_target * 0.85:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat        = aLocation2.lat - aLocation1.lat
    dlong       = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def main():
    altitude_target     = 2.0
    # Set up velocity mappings
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend
    vel_x = [2, 2, -2, -2]
    vel_y = [2,-2, -2, 2]
    vel_z = -0.5
    duration = 2
    print("Input takeoff")
    while input('input ')!= 'takeoff':
        print("Input takeoff")
        print("baro -- ",vehicle.location.global_relative_frame.alt)
        print("lidar -- ",vehicle.rangefinder.distance)
        print("Mode -- ",vehicle.mode)

    arm_and_tekoff(altitude_target)
    vehicle.flush()
    time.sleep(1)
    
    try:
        for i in range(len(vel_x)):
            print(str(i) + " goto x = " + str(vel_x[i]) + "| y = " + str(vel_y[i]))
            send_ned_velocity(vel_x[i],vel_y[i],vel_z,duration)
            time.sleep(10)
        vehicle.flush()
        vehicle.mode = VehicleMode("LAND")
    except KeyboardInterrupt:
        print("Keyboard Interupt")
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
