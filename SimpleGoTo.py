from dronekit import connect
import argparse as ap
import time
import waypoints as wp

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
        if current_altitude >= aTargetAltitude * 0.85:
            print("Reached target altitude")
            break
        time.sleep(1)

def getWP(lat_home = 0.0, lon_home = 0.0, alt_home = 0.0):
    waypoints   = wp.setup()

    if lat_home != 0.0 and lon_home != 0.0 and alt_home != 0.0:
        wp.set_home(lat_home, lon_home, alt_home)
    else:
        lat_home = wp.get_home()[0]
        lon_home = wp.get_home()[1]
        alt_home = wp.get_home()[2]

    return lat_home, lon_home, alt_home, waypoints

def main():
    altitude_target     = 2.0
    while input('input ')!= 'takeoff':
        print("Input takeoff")
        print("baro -- ",vehicle.location.global_relative_frame.alt)
        print("lidar -- ",vehicle.rangefinder.distance)
        print("Mode -- ",vehicle.mode)

    arm_and_tekoff(altitude_target)
    lat_home, lon_home, alt_home, waypoints = getWP
    vehicle.flush()
    time.sleep(1)

    try:
        for i in range(len(waypoints)):
            index           = waypoints[i]
            lat_target      = index[0]
            lon_target      = index[1]

            location_target = LocationGlobalRelative(lat_target,lon_target,vehicle.location.global_relative_frame.alt)
            print('goto ' + location_target)
            vehicle.simple_goto(location_target)
        
        vehicle.mode = VehicleMode('RTL')

    except KeyboardInterrupt:
        print('ERROR')

    if sitl:
        sitl.stop

    vehicle.flush()
    vehicle.close() 