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

def degreesToRadians(degrees): #function to convert degrees to radians, required for conversion of gps coordinates into distance
    return degrees * math.pi / 180;

def distanceInmBetweenEarthCoordinates(lat1, lon1, lat2, lon2): #function to calculate distance between two gps coordinates
    earthRadiusm = 6378137;#radius of the earth in meters
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
        waypoint[inc]= {
            'lat': cmd.x,
            'lon': cmd.y,
            'alt': cmd.z,
            'command': cmd.command
        }
        inc=inc+1
    #print(waypoint)
    #p = requests.get("http://127.0.0.1:3000/waypoints", json=waypoint)
    p = requests.get("https://nicwebpage.herokuapp.com/waypoints",json=json.dumps(waypoint))


dist=[]
def calculate_dist():
    total_dist=0
    for i in range(len(waypoint)-1):
        try:
            dist.append(distanceInmBetweenEarthCoordinates(waypoint[i+1]['lat'],waypoint[i+1]['lon'],waypoint[i+2]['lat'],waypoint[i+2]['lon']))
            total_dist += dist[i]
        except Exception as e:
            #print(e)
            pass
    return total_dist

checker=False
check = True
total=0
def send_data(threadName, delay):
    global checker
    global check
    global total
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
            if not check:
                try:
                    time2 = datetime.datetime.now()
                    flight_time=float((time2-time1).total_seconds())
                    vel=float(vehicle.groundspeed)
                    #print("flight time",flight_time)
                    #print("total distance",total)
                    est=float((float(total)-flight_time*3)/3+13+len(waypoint))

                    if est<=0:
                        est=0
                    data["est"]=est
                    #print("estimated time:", est)
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
        





def start():
    try:
        thread.start_new_thread(send_data,("Send Data", 1))
        save_mission()
        calculate_dist()
    except Exception as e:
        pass




start()
#print("total distance:", total)
while 1:
    pass
    #save_mission()

        #print("estimated time: ",(total-dist_travel)/vehicle.groundspeed)

        #print(e.args)
        #pass
