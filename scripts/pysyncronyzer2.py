# this script is used to sync the BAGs recorded with these published topics:
# 
# /canbus/data
# /clock
# /gps_trimble/trimble/position
# /rosout
# /rosout_agg
# /sync_image
# /trimble/fix
# /trimble/gga
# /vectornav/IMU
# /velodyne_points
# 
# where /sync_lidar is missing. Moreover, in the first 4 bags recorded in july 2023 (see below) sometimes a frame is missing. I used this node to sincronize
# 
# invett_c4__2023-07-25-17-24-47.bag
# invett_c4__2023-07-25-17-29-59.bag
# invett_c4__2023-07-25-17-32-24.bag
# invett_c4__2023-07-25-17-35-58.bag

# Create PCD and IMAGES with.
# PCD header stamp > has microseconds
# ROS header stamp > has nanoseconds 
# rosrun c4 pointcloud_to_pcd input:=/tosave_sync_lidar _binary:=True _filename_format:=_%010d.pcd _compressed:=True
# rosrun c4 image_saver image:=/tosave_sync_image _filename_format:=_%010d.png _stamped_filename:='True'  

# rosrun gps_time  gps_time_fixer           #fixes both gps + utm topics (/trimple/fix and /gps_trimble/trimble/position) republish same topic with w_time suffix
# rosrun c4 gps_utm_logger.py

# Create videos with:
# rosrun image_view video_recorder _codec:=X264 _filename:=invett.mp4 image:=/tosave_sync_image
# rosrun image_view video_recorder _codec:=fmp4 _filename:=invett.mp4 image:=/tosave_sync_image

#  /gps_trimble/trimble/position --> x/y/z (cartesian)
#  /trimble/fix                  --> lat/lon/altitude

import argparse
import rospy
import message_filters
from std_msgs.msg import Int32, Float32

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

# https://github.com/Turbo87/utm/blob/master/utm/conversion.py#L4
try:
    import numpy as mathlib
    use_numpy = True
except ImportError:
    import math as mathlib
    use_numpy = False

km_h=30
slop=0.01
window=50
input_lidar_topic='/velodyne_points'
input_image_topic='/sync_image'
input_LATLON_fix_topic='/trimble/fix_w_time'    #for july23 bags, use gps_time_fixer -- convert here with from_latlon or subscribe to /gps_trimble/trimble/position_w_time'
input_CARTESIAN_fix_topic='/gps_trimble/trimble/position_w_time'

output_lidar_topic='tosave_sync_lidar'
output_image_topic='tosave_sync_image'
output_gps_cartesian='tosave_gps_cartesian'
output_gps_latlon='tosave_gps_latlon'

# FOLLOWING STUFF IS FROM https://github.com/Turbo87/utm/blob/master/utm/conversion.py#L41 TO CONVERT LAT LON TO UTM COORDINATES (EASTING+NORTHING)
ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
K0 = 0.9996
E = 0.00669438
E2 = E * E
E3 = E2 * E
E_P2 = E / (1 - E)
SQRT_E = mathlib.sqrt(1 - E)
_E = (1 - SQRT_E) / (1 + SQRT_E)
_E2 = _E * _E
_E3 = _E2 * _E
_E4 = _E3 * _E
_E5 = _E4 * _E
M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
M3 = (15 * E2 / 256 + 45 * E3 / 1024)
M4 = (35 * E3 / 3072)
P2 = (3 / 2 * _E - 27 / 32 * _E3 + 269 / 512 * _E5)
P3 = (21 / 16 * _E2 - 55 / 32 * _E4)
P4 = (151 / 96 * _E3 - 417 / 128 * _E5)
P5 = (1097 / 512 * _E4)
R = 6378137
    
def from_latlon(latitude, longitude, force_zone_number=None, force_zone_letter=None):
    """This function converts Latitude and Longitude to UTM coordinate

        Parameters
        ----------
        latitude: float or NumPy array
            Latitude between 80 deg S and 84 deg N, e.g. (-80.0 to 84.0)

        longitude: float or NumPy array
            Longitude between 180 deg W and 180 deg E, e.g. (-180.0 to 180.0).

        force_zone_number: int
            Zone number is represented by global map numbers of an UTM zone
            numbers map. You may force conversion to be included within one
            UTM zone number.  For more information see utmzones [1]_

        force_zone_letter: str
            You may force conversion to be included within one UTM zone
            letter.  For more information see utmzones [1]_

        Returns
        -------
        easting: float or NumPy array
            Easting value of UTM coordinates

        northing: float or NumPy array
            Northing value of UTM coordinates

        zone_number: int
            Zone number is represented by global map numbers of a UTM zone
            numbers map. More information see utmzones [1]_

        zone_letter: str
            Zone letter is represented by a string value. UTM zone designators
            can be accessed in [1]_


       .. _[1]: http://www.jaworski.ca/utmzones.htm
    """
    if not in_bounds(latitude, -80, 84):
        raise OutOfRangeError('latitude out of range (must be between 80 deg S and 84 deg N)')
    if not in_bounds(longitude, -180, 180):
        raise OutOfRangeError('longitude out of range (must be between 180 deg W and 180 deg E)')
    if force_zone_number is not None:
        check_valid_zone(force_zone_number, force_zone_letter)

    lat_rad = mathlib.radians(latitude)
    lat_sin = mathlib.sin(lat_rad)
    lat_cos = mathlib.cos(lat_rad)

    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2

    if force_zone_number is None:
        zone_number = latlon_to_zone_number(latitude, longitude)
    else:
        zone_number = force_zone_number

    if force_zone_letter is None:
        zone_letter = latitude_to_zone_letter(latitude)
    else:
        zone_letter = force_zone_letter

    lon_rad = mathlib.radians(longitude)
    central_lon = zone_number_to_central_longitude(zone_number)
    central_lon_rad = mathlib.radians(central_lon)

    n = R / mathlib.sqrt(1 - E * lat_sin**2)
    c = E_P2 * lat_cos**2

    a = lat_cos * mod_angle(lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a

    m = R * (M1 * lat_rad -
             M2 * mathlib.sin(2 * lat_rad) +
             M3 * mathlib.sin(4 * lat_rad) -
             M4 * mathlib.sin(6 * lat_rad))

    easting = K0 * n * (a +
                        a3 / 6 * (1 - lat_tan2 + c) +
                        a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000

    northing = K0 * (m + n * lat_tan * (a2 / 2 +
                                        a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c**2) +
                                        a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))

    if mixed_signs(latitude):
        raise ValueError("latitudes must all have the same sign")
    elif negative(latitude):
        northing += 10000000

    return easting, northing, zone_number, zone_letter
def latitude_to_zone_letter(latitude):
    # If the input is a numpy array, just use the first element
    # User responsibility to make sure that all points are in one zone
    if use_numpy and isinstance(latitude, mathlib.ndarray):
        latitude = latitude.flat[0]

    if -80 <= latitude <= 84:
        return ZONE_LETTERS[int(latitude + 80) >> 3]
    else:
        return None
def in_bounds(x, lower, upper, upper_strict=False):
    if upper_strict and use_numpy:
        return lower <= mathlib.min(x) and mathlib.max(x) < upper
    elif upper_strict and not use_numpy:
        return lower <= x < upper
    elif use_numpy:
        return lower <= mathlib.min(x) and mathlib.max(x) <= upper
    return lower <= x <= upper
def latlon_to_zone_number(latitude, longitude):
    # If the input is a numpy array, just use the first element
    # User responsibility to make sure that all points are in one zone
    if use_numpy:
        if isinstance(latitude, mathlib.ndarray):
            latitude = latitude.flat[0]
        if isinstance(longitude, mathlib.ndarray):
            longitude = longitude.flat[0]

    if 56 <= latitude < 64 and 3 <= longitude < 12:
        return 32

    if 72 <= latitude <= 84 and longitude >= 0:
        if longitude < 9:
            return 31
        elif longitude < 21:
            return 33
        elif longitude < 33:
            return 35
        elif longitude < 42:
            return 37

    return int((longitude + 180) / 6) + 1       
def zone_number_to_central_longitude(zone_number):
    return (zone_number - 1) * 6 - 180 + 3    
def mod_angle(value):
    """Returns angle in radians to be between -pi and pi"""
    return (value + mathlib.pi) % (2 * mathlib.pi) - mathlib.pi
def mixed_signs(x):
    return use_numpy and mathlib.min(x) < 0 and mathlib.max(x) >= 0
def negative(x):
    if use_numpy:
        return mathlib.max(x) < 0
    return x < 0
# END OF THE STUFF 


def callback(lidar, image, gps_time):
    rospy.loginfo_once('First message received. No other message will be printed. You can change log level to debug to see some more info.')
    diff=(abs(lidar.header.stamp-image.header.stamp).to_sec())
    rospy.loginfo("lidar %d.%d\t image: %d.%d\t gps %d.%d\t diff[lidar_image]: %.9fs\t distance %d km/h: %fm", 
                  lidar.header.stamp.secs, 
                  lidar.header.stamp.nsecs, 
                  image.header.stamp.secs,
                  image.header.stamp.nsecs, 
                  gps_time.header.stamp.secs,
                  gps_time.header.stamp.nsecs,
                  diff, 
                  km_h, 
                  diff*km_h/3.6)
    syncpub_lidar.publish(lidar)
    syncpub_image.publish(image)   
    syncpub_cartesian.publish(gps_time)
    
    # easting, northing, zone_number, zone_letter = from_latlon(latitude=gps_time.latitude, longitude=gps_time.longitude)

    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--kmh', dest='kmh', type=int, default=30, help='set the speed used to evaluate displacement')
    parser.add_argument('--slop', dest='slop', type=float, default=0.01, help='windows in seconds for syncronization')
    parser.add_argument('--window', dest='window', type=int, default=50, help='window for message syncronization')
    parser.add_argument('--input_lidar_topic', dest='input_lidar_topic', type=str, default=input_lidar_topic, help='velodyne point cloud topic')
    parser.add_argument('--input_image_topic', dest='input_image_topic', type=str, default=input_image_topic, help='velodyne point cloud topic')
    
    args, unknown = parser.parse_known_args()
    km_h = args.kmh
    slop = args.slop
    window = args.window
    input_lidar_topic = args.input_lidar_topic
    input_image_topic = args.input_image_topic    
    
    rospy.init_node('pysyncronizer2', anonymous=True)  

    unsync_lidar = message_filters.Subscriber(input_lidar_topic, PointCloud2)
    unsync_image = message_filters.Subscriber(input_image_topic, Image)
    unsync_gps_cartesian = message_filters.Subscriber(input_CARTESIAN_fix_topic, PointStamped)
    unsync_gps_latlon = message_filters.Subscriber(input_LATLON_fix_topic, NavSatFix)

    syncpub_lidar = rospy.Publisher(output_lidar_topic, PointCloud2, queue_size=1)
    syncpub_image = rospy.Publisher(output_image_topic, Image, queue_size=1)
    syncpub_cartesian = rospy.Publisher(output_gps_cartesian, PointStamped, queue_size=1)
    syncpub_lon = rospy.Publisher(output_gps_latlon, NavSatFix, queue_size=1)

    #ts = message_filters.ApproximateTimeSynchronizer([unsync_lidar, unsync_image], window, slop, allow_headerless=False)
    ts = message_filters.ApproximateTimeSynchronizer([unsync_lidar, unsync_image, unsync_gps_cartesian], window, slop, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.loginfo_once("Hey! pysyncronizer2 is up&running, listening to %s and %s.", input_lidar_topic, input_image_topic)
    
    rospy.loginfo_once("kmh..................%s", km_h)
    rospy.loginfo_once("slop.................%s", slop)
    rospy.loginfo_once("window...............%s", window)      
    
    rospy.loginfo_once("Waiting for messages, press ctrl+c to kill this node ...")
    rospy.spin()

    print("\nGoodbye!")
