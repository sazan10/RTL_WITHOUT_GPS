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
from socketIO_client_nexus import SocketIO, BaseNamespace
from requests.exceptions import ConnectionError#
import numpy as np
import mission as mi
import distance as dis
#Set up option parsing to get connection string
import argparse

#parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
#parser.add_argument('--connect',
#                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
#args = parser.parse_args()

#connection_string = args.connect
sitl = None

#print("\nConnecting to vehicle on: ,s" , connection_string)

#vehicle = connect(connection_string, wait_ready=True)d={}
def read_username_password():
    d={}
    r=0
    file = open('pyfile.txt', 'r')
    for line in file:
    	d[r]=line.split("\n")[0]
    	r=r+1
    return d


try:
    socket1 = SocketIO('http://192.168.1.119', 3000, wait_for_connection=False)#establish socket connection to desired server
    socket = socket1.define(BaseNamespace,'/pulchowk')
    #socket = SocketIO('https://nicwebpage.herokuapp.com', verify =False)
    socket.emit("joinPiPulchowk")
    #socket.emit("joinPi")
    #socket.emit("usernamePassword",read_username_password())
except ConnectionError:
    print('The server is down. Try again later.')

try:
    vehicle = connect('127.0.0.1:14551', wait_ready=True)
except Exception as e:
    error={'context':'Connection','msg':'Connection problem with pixhawk'}
    socket.emit("error",error)
vehicle.wait_ready('autopilot_version')


waypoint={}
def on_mission_download(var): #this function is called once the server requests to download the mission file, to send mission to server
    print("DOWNLOAD MISSION COMMAND BY USER",var)
    if bool(waypoint):#checking if the mission file has been downloaded from pixhawk
        socket.emit('waypoints',waypoint)
        print("Mission downloaded by user")
    else:
        error={'context':'GPS/Mission','msg':'GPS error OR no mission file received!!'}
        socket.emit("error",error)



flight_checker=False #to check that the fly command is given successfully only once
def on_initiate_flight(var):
    global flight_checker #reuired global, as this is a socket funciton, got no idea how to pass parameters to socket function
    try:
        if vehicle.is_armable and not flight_checker: #checking if vehicle is armable and fly command is genuine
            socket.emit("success","flight_success")
            print("FLIGHT INITIATED BY USER")
            arm.arm_and_takeoff(vehicle,2) #arm and takeoff upto 2 meters
            vehicle.mode = VehicleMode("AUTO") #switch vehicle mode to auto
            flight_checker=True ## True if succesful flight, no further flight commands will be acknowledged
    except Exception as e:
        #print(e)
        error={'context':'Prearm','msg':'Pre-arm check failed!!!'}
        socket.emit("error",error)

# Get all vehicle attributes (state)
print("\nGet all vehicle attribute values:")



def send_data(threadName, delay):
    global waypoint #needs to be global as it is accessed by on_mission_download, socket function
    checker=False #flag to start the functions to calculate total distance and eta once the mission is received from pixhawk
    total=0
    check = True
    divisor=1
    last_vel=0
    data = {}
    while 1:
        #print("data sending ")
        try:
            loc = vehicle.location.global_frame
            data["numSat"] = vehicle.gps_0.satellites_visible
            data["hdop"] = vehicle.gps_0.eph
            data["fix"] = vehicle.gps_0.fix_type
            data["lat"] = format(loc.lat,'.15f')
            data["lng"] = format(loc.lon,'.15f')
        except Exception as e:
            pass
        vel = vehicle.velocity
        status = str(vehicle.system_status)
        data["conn"] = 'True'
        data["arm"] = vehicle.armed
        data["ekf"] = vehicle.ekf_ok
        data["mode"] = vehicle.mode.name
        data["alt"] = format(loc.alt, '.2f')
        data["altr"] = vehicle.location.global_relative_frame.alt
        data["head"] = format(vehicle.heading, 'd')
        data["as"]=format(vehicle.airspeed, '.3f')
        data["lidar"] = vehicle.rangefinder.distance
        data["gs"] = format(vehicle.groundspeed, '.3f')
        data["status"] = status[13:]
        data["volt"] = format(vehicle.battery.voltage, '.2f')
        print(datetime.datetime.now())
        if  checker:#only if waypoints are successfully read

            if vehicle.armed and check: # check is used to receive time of arming only once, at first arming
                time1 = datetime.datetime.now() #receive time once armed
                check=False
            if not check :
                try:
                    time2 = datetime.datetime.now()# calculate current time
                    flight_time=float((time2-time1).total_seconds()) # calculate total flight time
                    vel=float((last_vel+vehicle.groundspeed)/divisor) #calculate the average velocity upto current time
                    est=float((float(total)-flight_time*vel)/3.5) #estimate eta, with assummed velocity 3.5 m/s
                    if est<=0.5: #making sure eta is not less thatn 0.5
                        est=0
                    data["est"]=est
                    last_vel+=vehicle.groundspeed #summing up velcity to average them
                    divisor+=1 # the number of values of velocity
                except Exception as e:
                    pass

        socket.emit('data',data) #send data to server
        socket.on('mission_download',on_mission_download) #keep listening for download command for mission from server
        socket.on('initiate_flight',on_initiate_flight)#keep listening for fly command from user
        print(data["head"])

        if vehicle.gps_0.fix_type > 1   and not checker: ## check gps before downloading the mission as home points are not received without gps, and checker to download mission only once
            try:
                waypoint=mi.save_mission(vehicle) # call the save mission function which downloads the mission from pixhawk and arranges waypoints into a dictionary
                print("MISSION DOWNLOADED")
                total=dis.calculate_dist(waypoint) # calculate total distance between home and final waypoint
                socket.emit('homePosition',waypoint[0]) # send homePosition to server as soon as gps lock
                checker=True # switch value of checker such that mission is downloaded only once from pixhawk, and condition to calculate eta is met
            except Exception as e:
                print(e)
                print("GPS error OR no mission file received!!!")
        socket1.wait(seconds=0.2) #sends or waits for socket activities in every seconds specified
                #socket.wait()


def start(): #to perform all the operations in a thread
    try:
        print("inside start")
        thread.start_new_thread(send_data,("Send Data", 1)) # function, arguments
        #save_mission()
        #calculate_dist()
    except Exception as e:
        pass


start()
while True:

    pass
