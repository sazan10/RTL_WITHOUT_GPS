

def download_mission(vehicle):
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


def save_mission(vehicle):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    #Download mission from vehicle
    missionlist = download_mission(vehicle)
    #Add file-format information

    #Add home location as 0th waypoint
    waypoint={}
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
    return waypoint
