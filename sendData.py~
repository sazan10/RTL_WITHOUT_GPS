from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import datetime
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
data = {}
# Get all vehicle attributes (state)
print("\nGet all vehicle attribute values:")


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

def degreesToRadians(degrees): #function to convert degrees to radians, required for conversion of gps coordinates into distance
    return degrees * math.pi / 180;

def distanceInmBetweenEarthCoordinates(lat1, lon1, lat2, lon2): #function to calculate distance between two gps coordinates
    #earthRadiusm = 6378137;#radius of the earth in meters
    earthRadiusm=6371000
    dLat = degreesToRadians(lat2-lat1)
    dLon = degreesToRadians(lon2-lon1)
    latt1 = degreesToRadians(lat1)
    latt2 = degreesToRadians(lat2)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(latt1) * math.cos(latt2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return earthRadiusm * c

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


waypoint = {}

def save_mission():
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information

    #Add home location as 0th waypoint

    home=vehicle.home_location
    #print(home.lat,home.lon,home.alt)
    waypoint[0]={
    'lat':home.lat,
    'lon':home.lon,
    'alt':home.alt
    }
    #Add commands
    inc=1
    for cmd in missionlist:
        if cmd.command!=22:
            waypoint[inc]= {
            'lat': cmd.x,
            'lon': cmd.y,
            'alt': cmd.z,
            'command': cmd.command
            }
            inc=inc+1
    #print(waypoint)
    #p = requests.get("http://127.0.0.1:3000/waypoints", json=waypoint)
    p = requests.get("https://nicwebpage.herokuapp.com/waypoints",json=waypoint)


dist=[]
def calculate_dist():
    total_dist=0
    for i in range(len(waypoint)-1):
        try:
            dist.append(distanceInmBetweenEarthCoordinates(waypoint[i]['lat'],waypoint[i]['lon'],waypoint[i+1]['lat'],waypoint[i+1]['lon']))
            #print(i,dist[i])
            total_dist += dist[i]
        except Exception as e:
            #print(e)
            pass
    return total_dist

checker=False
check = True
total=0
divisor=1
last_vel=0
def send_data(threadName, delay):
    global checker
    global check
    global total
    global divisor
    global last_vel
    print("started")
    while 1:
        loc = vehicle.location.global_frame
        vel = vehicle.velocity
        status = str(vehicle.system_status)
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
        data["as"]=format(vehicle.airspeed, '.3f')
        data["lidar"] = vehicle.rangefinder.distance
        data["gs"] = format(vehicle.groundspeed, '.3f')
        #data["roll"] = format(vehicle.attitude.roll, '.2f')
        #data["pitch"] = format(vehicle.attitude.pitch, '.2f')
        #data["yaw"] = format(vehicle.attitude.yaw, '.2f')
        data["status"] = status[13:]
        data["volt"] = format(vehicle.battery.voltage, '.2f')
        #data["vx"] = vel[0]
        #data["vy"] = vel[1]
        #data["vz"] = vel[2]
        data["heartbeat"] = vehicle.last_heartbeat
        data["numSat"] = vehicle.gps_0.satellites_visible
        data["hdop"] = vehicle.gps_0.eph
        data["fix"] = vehicle.gps_0.fix_type
        #print(data)
        # for key,value in vehicle.parameters.iteritems():
        #     print ("Key: %s value: %s",key,value)
        #print(r.text)
        #time.sleep(delay)
        #print(len(waypoint))

        if(checker):
            if vehicle.armed and check:
                time1 = datetime.datetime.now()
                check=False
            if not check :
                try:
                    time2 = datetime.datetime.now()
                    flight_time=float((time2-time1).total_seconds())
                    vel=float((last_vel+vehicle.groundspeed)/divisor)

                    #print("flight time",flight_time)
                    #print("total distance",total)
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
                    #print(e.args)
                    pass

        #r = requests.get("http://127.0.0.1:3000/data",params=data)
        r = requests.get("https://nicwebpage.herokuapp.com/data",params=data)
        #r = requests.get("http://photooverlay.com/nic/get_data.php",params=data)

        if r.text == '1':
        #if not checker:
            save_mission()
            checker=True
            total=calculate_dist()
            print("total:",total)






def start():
    try:
        thread.start_new_thread(send_data,("Send Data", 1))
        save_mission()
        #calculate_dist()
    except Exception as e:
        pass





#print("total distance:", total)
initiator={}
initiator["flight"]=0
initiator_flag=False
while 1:
    if not initiator_flag:
        initiate_flight=requests.get("https://nicwebpage.herokuapp.com/flight",params=initiator)
        print (initiate_flight.text)
        if initiate_flight.text=='1':
            start()
            vehicle.mode = VehicleMode("GUIDED")
            arm_and_takeoff()
            initiator_flag=True

    #save_mission()

        #print("estimated time: ",(total-dist_travel)/vehicle.groundspeed)

        #print(e.args)
        #pass
