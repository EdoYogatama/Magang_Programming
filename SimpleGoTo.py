from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse as ap
import time
import waypoints as wp
import future
import math

# Argument Parser, command to connect to the vehicle
parser  = ap.ArgumentParser(description='Control Copter and send commend in GUIDED mode')
parser.add_argument('--connect', 
                    help='Vehicle connection target string. If not specified, SITL automatically started and used.')
args    = parser.parse_args()

connection_string   = args.connect
sitl                = None

waypoints           = 0

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

def getWP(lat_home = 0.0, lon_home = 0.0, alt_home = 0.0):
    wp.setup()
    lat_target = []
    lon_target = []

    waypoints  = len(wp.gets())    
    for i in range(waypoints):
        if i == 0:
            lat_home    = wp.get(i)[0]
            lon_home    = wp.get(i)[1]
        else:
            lat_target.append(wp.get(i)[0])
            lon_target.append(wp.get(i)[1])

    return lat_home,lon_home,lat_target,lon_target

def get_distance_metres(aLocation1, aLocation2):
    dlat        = aLocation2.lat - aLocation1.lat
    dlong       = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def main():
    altitude_target     = 2.0
    while input('input ')!= 'takeoff':
        print("Input takeoff")
        print("baro -- ",vehicle.location.global_relative_frame.alt)
        print("lidar -- ",vehicle.rangefinder.distance)
        print("Mode -- ",vehicle.mode)

    arm_and_tekoff(altitude_target)
    vehicle.flush()
    time.sleep(1)
    
    lat_home,lon_home,lat_target,lon_target = getWP()
    home    = LocationGlobalRelative(lat_home,lon_home,vehicle.location.global_relative_frame.alt)

    try:
        for i in range(len(lat_target)):

            currentlat = vehicle.location.global_relative_frame.lat
            currentlon = vehicle.location.global_relative_frame.lon
            
            location_target = LocationGlobalRelative(lat_target[i],lon_target[i],vehicle.location.global_relative_frame.alt)
            
            print('goto ' + str(location_target))
            
            vehicle.simple_goto(location_target)
            #time.sleep(30)
            
            while vehicle.mode == VehicleMode("GUIDED"):
                remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, location_target)
                print(remainingDistance)
                if remainingDistance <= 5: #Just below target, in case of undershoot.
                    print ("Reached target")
                    print("Drop")
                    break
                time.sleep(0.1)
                    
        vehicle.mode = VehicleMode('RTL')

    except KeyboardInterrupt:
        vehicle.mode = VehicleMode('RTL')
        print('ERROR')
    except SyntaxError:
        vehicle.mode = VehicleMode('RTL')
        print("There is error with your syntax, please check it again")

    if sitl:
        sitl.stop

    vehicle.flush()
    vehicle.close()

if __name__ == "__main__":
    main()
