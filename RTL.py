#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py:

Demonstrates how to get and set vehicle state and parameter information,
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math
import threading
import thread
import requests
from pymavlink import mavutil #
#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.

print("\nConnecting to vehicle on: ,s" , connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

# Get all vehicle attributes (state)
print("\nGet all vehicle attribute values:")
check=True
print(type(vehicle.location.global_frame))
#initial_battery = vehicle.battery
#final_battery=initial_battery
location_lat=[] #initializing array to store lattitudes after reaching each waypoint
location_lon=[] # initializing array to store longitudes after reaching each waypoint
location_lat.append(vehicle.location.global_frame.lat) #storing latitude of home/ drone's initial position
location_lon.append(vehicle.location.global_frame.lon) #sotring longitude of home/ drone's initial position
# while((initial_battery.level-final_battery.level)<10):
#     final_battery=vehicle.battery
    #print(initial_battery.level-final_battery.level)
#while(True):
#    print(vehicle.commands.next)

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------


#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------
def send_data(threadName, delay):
    while 1:
        loc = vehicle.location.global_frame
        vel = vehicle.velocity
        status = str(vehicle.system_status)
        data = {}
        data["firm"] = str(vehicle.version)
        data["conn"] = True
        data["arm"] = vehicle.armed
        data["ekf"] = vehicle.ekf_ok
        data["mode"] = vehicle.mode.name
        data["lat"] = format(loc.lat,'.15f')
        data["long"] = format(loc.lon,'.15f')
        data["alt"] = format(loc.alt, '.2f')
        data["altr"] = vehicle.location.global_relative_frame.alt
        data["head"] = format(vehicle.heading, 'd')
        data["lidar"] = vehicle.rangefinder.distance
        data["gs"] = format(vehicle.groundspeed, '.3f')
        data["roll"] = format(vehicle.attitude.roll, '.2f')
        data["pitch"] = format(vehicle.attitude.pitch, '.2f')
        data["yaw"] = format(vehicle.attitude.yaw, '.2f')
        data["status"] = status[13:]
        data["volt"] = format(vehicle.battery.voltage, '.2f')
        data["vx"] = vel[0]
        data["vy"] = vel[1]
        data["vz"] = vel[2]
        data["heartbeat"] = vehicle.last_heartbeat
        data["numSat"] = vehicle.gps_0.satellites_visible
        data["hdop"] = vehicle.gps_0.eph
        data["fix"] = vehicle.gps_0.fix_type
        print(data)
        r = requests.get("https://nicwebpage.herokuapp.com/data",params=data)
        #r = requests.get("http://photooverlay.com/nic/get_data.php",params=data)
        #print(r.text)
        time.sleep(delay)

def start():
    try:
        print("hello2")
        thread.start_new_thread(send_data,("Send Data", 1))
    except Exception as e:
        print(e)
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------
print("hello")
start()

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

def degreesToRadians(degrees): #function to convert degrees to radians, required for conversion of gps coordinates into distance
    return degrees * math.pi / 180;

def distanceInmBetweenEarthCoordinates(lat1, lon1, lat2, lon2):# function to calculate distance between two gps coordinates
    earthRadiusm = 6378137;#radius of the earth in meters
    dLat = degreesToRadians(lat2-lat1);
    dLon = degreesToRadians(lon2-lon1);
    latt1 = degreesToRadians(lat1);
    latt2 = degreesToRadians(lat2);
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(latt1) * math.cos(latt2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a));
    return earthRadiusm * c;

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------





#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

def condition_yaw(heading, relative=False):#function to maintain the yaw condition

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")
# We have a home location, so print it!
print("\n Home location: ,s" , vehicle.home_location)

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------





#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

def arm_and_takeoff(aTargetAltitude):
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
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------




#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------
def distance_to_current_waypoint():#this function calculates the distance to the current waypoint it is traversing to
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------



#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):#send velocity commands ,x=north,south , y = east, west , z= up/down to move the drone

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

#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------


#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------


#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------

waypoint_reached=0
vel_x = []# initializing array to store x(NS), velocity at each waypoint
vel_y = []#initializingarray to store y(EW), velocity at each waypoint

while((vehicle.commands.next<=6)):#checking while batttery level does not drop by 25%
    if(vehicle.commands.next>2 and waypoint_reached!=vehicle.commands.next):# it should be greater or equal to 2 because takeoff is always considered 1st, for which we do not need to calculate distance, waypoint!=vehicle.commands.next means when next waypoint is reached
        [final_vel_x,final_vel_y,final_vel_z]=vehicle.velocity
        location_lat.append(vehicle.location.global_frame.lat)
        location_lon.append(vehicle.location.global_frame.lon)
        vel_x.append(final_vel_x)
        vel_y.append(final_vel_y)
        final_battery=vehicle.battery
        #print(waypoint,"   ",vehicle.commands.next)
        waypoint_reached=vehicle.commands.next
[final_vel_x2,final_vel_y2,final_vel_z2]=vehicle.velocity
vel_x.append(final_vel_x2)
vel_y.append(final_vel_y2)
location_lat.append(vehicle.location.global_frame.lat)
location_lon.append(vehicle.location.global_frame.lon)
print("vel_x",vel_x)
print("vel_y",vel_y)
print("latitudes",location_lat)
print("longitudes",location_lon)
leng=len(location_lat)
print(leng)
print(len(vel_x))
vehicle.mode = VehicleMode("GUIDED")
for i in range(leng,1,-1):
    dis=distanceInmBetweenEarthCoordinates(location_lat[i-1],location_lon[i-1],location_lat[i-2],location_lon[i-2])
    vel=math.sqrt(vel_x[i-2]*vel_x[i-2]+vel_y[i-2]*vel_y[i-2])
    times=(dis)/vel
    print("vel_x",vel_x[i-2]," vel_y",vel_y[i-2])
    print("lat_lon",location_lat[i-1],location_lon[i-1],location_lat[i-2],location_lon[i-2])
    print(i, "now return cmd for each , distance= ", dis)
    print("time",times)
    #arm_and_takeoff(vehicle.location.global_relative_frame.alt)
    send_ned_velocity(-vel_x[i-2],-vel_y[i-2],0,int(times))

print(dis)
print(vel_x)
#----------------------------------------------------------------------------------
#----------------------------------------------------------------------------------







# [final_vel_x,final_vel_y,final_vel_z]=vehicle.velocity
# final=vehicle.location.global_frame
# dis=distanceInmBetweenEarthCoordinates(initial.lat,initial.lon,final.lat,final.lon)
# condition_yaw(-vehicle.heading)
# vel=math.sqrt(final_vel_x*final_vel_x + final_vel_y*final_vel_y)
# timed=(dis)/vel
# print("Final_velocity_X:",final_vel_x)
# print("Final_velocity_Y:",final_vel_y)
# print("time",timed)
# print(dis)
# arm_and_takeoff(vehicle.location.global_relative_frame.alt)
# send_ned_velocity(int(-final_vel_x),int(-final_vel_y),0,int(timed))
vehicle.mode=VehicleMode("LAND")
print("\nClose vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
