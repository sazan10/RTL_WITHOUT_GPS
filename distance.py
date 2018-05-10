
import math


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


def calculate_dist(waypoint):
    dist=[]
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
