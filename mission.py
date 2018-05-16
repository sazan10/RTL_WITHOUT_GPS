

def save_mission(vehicle):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    #output='QGC WPL 110\n'
    for cmd in cmds:
        missionlist.append(cmd)

    #Add file-format information

    #Add home location as 0th waypoint
    waypoint={}

    home=vehicle.home_location
        #print(home.lat,home.lon,home.alt)
    waypoint[0]={
    'lat':home.lat,
    'lng':home.lon,
    
    }
        #Add commands
    inc=1
    for cmd in missionlist:
        if cmd.command!=22:
            waypoint[inc]= {
            'lat': cmd.x,
            'lng': cmd.y,
            'alt': cmd.z,
            'command': cmd.command
            }
            inc=inc+1

    return waypoint
