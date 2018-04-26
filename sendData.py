from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math
import threading
import json
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
        thread.start_new_thread(send_data,("Send Data", 1))
    except Exception as e:
        print(e)

start()
while 1:
    pass
