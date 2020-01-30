import waypoints as wp
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse as ap
import time
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

def distanceBetween(lat1, long1, lat2, long2):
  ''' returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers'''
  
  delta = math.radians(long1-long2)
  sdlong = math.sin(delta)
  cdlong = math.cos(delta)

  lat1 = math.radians(lat1)
  lat2 = math.radians(lat2)

  slat1 = math.sin(lat1)
  clat1 = math.cos(lat1)
  slat2 = math.sin(lat2)
  clat2 = math.cos(lat2)
  
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
  delta = delta*delta
  delta += ((clat2 * sdlong)*(clat2 * sdlong))
  delta = math.sqrt(delta)
  denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
  delta = math.atan2(delta, denom)
  
  return delta * 6372795
'''
def toDeg(lat1,lon1,lat2,lon2):
    dlon = distanceBetween(lat1,0.0,lat2,0.0)
    dist = distanceBetween(lat1,lon1,lat2,lon2)
    degree     = math.degrees(vehicle.attitude.yaw)

    print(dlon)
    print(dist)
    print((dist/dlon))
    alpha = math.acos(dist/dlon-degree)

    return alpha
'''
def courseTo(lat1, long1,lat2,long2):
  ''' returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers'''
  
  dlon = math.radians(long2-long1)
  lat1 = math.radians(lat1)
  lat2 = math.radians(lat2)
  a1 = math.sin(dlon) * math.cos(lat2)
  a2 = math.sin(lat1) * math.cos(lat2) * math.cos(dlon)

  a2 = math.cos(lat1) * math.sin(lat2) - a2
  a2 = math.atan2(a1, a2)
  
  if (a2 < 0.0) :
    a2 += math.pi
  
  return math.degrees(a2)

def convert(lat1,lon1,lat2,lon2,duration):
    distance   = distanceBetween(lat1,lon1,lat2,lon2)
    degreewp     = courseTo(lat1,lon1,lat2,lon2)

    #print("degree",degree)
    print("degreewp",degreewp)

    output_x = (distance/duration) * math.cos(math.radians(degreewp))
    output_y = (distance/duration) * math.sin(math.radians(degreewp))
    '''
    output_x = distance * math.cos(degreewp)
    output_y = distance * math.sin(degreewp)
    '''
    print("NED-->",output_x,output_y)

    return output_x,output_y


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
    
    duration    = 1

    lat_home,lon_home,lat_target,lon_target = getWP()
    home    = LocationGlobalRelative(lat_home,lon_home,vehicle.location.global_relative_frame.alt)
    
    try:
        for i in range(len(lat_target)):
            if i == 0:
                vel_x,vel_y = convert(lat_home,lon_home,lat_target[i],lon_target[i],duration)
            else:
                vel_x,vel_y = convert(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,
                                        lat_target[i],lon_target[i],duration)
            
            send_ned_velocity(vel_y,vel_x,0,duration)
            print("goto " + str(vel_x) + " " + str(vel_y))
            
              distancenow   = distanceBetween(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_target[i],lon_target[i])

            while distancenow > 10:
                distancenow   = get_distance_metres(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_target[i],lon_target[i])
                print(distancenow)
                send_ned_velocity(vel_y,vel_x,0,duration)
                print("goto " + str(vel_x) + " " + str(vel_y))
                time.sleep(0.2)   
                
            time.sleep(5)

        vehicle.mode = VehicleMode("RTL")
    except KeyboardInterrupt:
        print("Krybord Interrupt")
    
if __name__ == "__main__":
    main()
