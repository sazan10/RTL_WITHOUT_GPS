from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import arm_takeoff as arm
import datetime
import math
import threading
import json
import thread
import time
import requests
from pymavlink import mavutil
from socketIO_client_nexus import SocketIO
from requests.exceptions import ConnectionError#
import numpy as np
import mission as mi
import distance as dis
#Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

print("\nConnecting to vehicle on: ,s" , connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

checker=False
total=0

waypoint = {}
def on_mission_download(var):
    global total
    global checker
    global waypoint
    print("DOWNLOAD MISSION COMMAND BY USER",var)
    waypoint=mi.save_mission(vehicle)
    try:
        socket.emit('waypoints',waypoint)
        print("MISSION SAVED BY USER")
    except Exception as e:
        print(args(e))
    total=dis.calculate_dist(waypoint)
    checker=True


try:
    socket = SocketIO('http://192.168.1.119', 3000, wait_for_connection=False)
    #socket = SocketIO('https://nicwebpage.herokuapp.com', verify =False)
    socket.emit("joinPi")
except ConnectionError:
    print('The server is down. Try again later.')


def on_initiate_flight(var):
    socket.emit("success","flight_success")
    print("FLIGHT INITIATED BY USER")
    arm.arm_and_takeoff(vehicle,5)
    vehicle.mode = VehicleMode("AUTO")




# Get all vehicle attributes (state)
print("\nGet all vehicle attribute values:")




check = True
divisor=1
last_vel=0
data = {}
def send_data(threadName, delay):
    global checker
    global check
    global total
    global divisor
    global last_vel

    while 1:
        #print("data sending ")
        loc = vehicle.location.global_frame
        vel = vehicle.velocity
        status = str(vehicle.system_status)
        data["conn"] = 'True'
        data["arm"] = vehicle.armed
        data["ekf"] = vehicle.ekf_ok
        data["mode"] = vehicle.mode.name
        data["lat"] = format(loc.lat,'.15f')
        data["long"] = format(loc.lon,'.15f')
        data["alt"] = format(loc.alt, '.2f')
        data["altr"] = vehicle.location.global_relative_frame.alt
        data["head"] = format(vehicle.heading, 'd')
        data["as"]=format(vehicle.airspeed, '.3f')
        data["lidar"] = vehicle.rangefinder.distance
        data["gs"] = format(vehicle.groundspeed, '.3f')
        data["status"] = status[13:]
        data["volt"] = format(vehicle.battery.voltage, '.2f')
        data["numSat"] = vehicle.gps_0.satellites_visible
        data["hdop"] = vehicle.gps_0.eph
        data["fix"] = vehicle.gps_0.fix_type
        print("total dist",total)
        if(checker):
            print("inside checker")
            if vehicle.armed and check:
                time1 = datetime.datetime.now()
                check=False
            if not check :
                try:
                    time2 = datetime.datetime.now()
                    flight_time=float((time2-time1).total_seconds())
                    vel=float((last_vel+vehicle.groundspeed)/divisor)

                    #print("flight time",flight_time)
                    print("total distance",total)
                    #print(total)
                    est=float((float(total)-flight_time*vel)/3.5)
                    #if est<=1 and (total-flight_time*vel)<=2:
                        #total=0
                    if est<=0.5:
                        est=0
                    data["est"]=est
                    #print(est)
                    #print("vel:",vel)
                    #print("estimated time:", est, total, flight_time*vel)
                    last_vel+=vehicle.groundspeed
                    divisor+=1
                except Exception as e:
                    #print(type(e))
                    print(e.args)
                    #pass

        socket.emit('data',data)
        socket.on('mission_download',on_mission_download)
        socket.on('initiate_flight',on_initiate_flight)
        socket.wait(seconds=0.1)

                #socket.wait()


def start():
    try:
        print("inside start")
        thread.start_new_thread(send_data,("Send Data", 1))
        #save_mission()
        #calculate_dist()
    except Exception as e:
        pass

flight_checker=False
#print("total distance:", total)

#while 1:
start()
while True:
    pass
