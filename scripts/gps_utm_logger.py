#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

counter = 0

def callback(data):
    global counter    
    filename=str(counter)
    filename=str(data.header.stamp)+'_'+(filename.rjust(10, '0')+'.txt')    
    f = open(filename, "w")    
    f.write(str(data.point.x)+','+str(data.point.y)+','+str(data.point.z))
    f.close()
    
    rospy.loginfo("Filename: %s \t easting %s \t northing %s", filename, data.point.x, data.point.y)
    
    counter = counter + 1

    
def listener():

    rospy.init_node('gps_utm_logger', anonymous=True)
    
    input_topic = "/gps_trimble/trimble/position_w_time"

    rospy.Subscriber(input_topic, PointStamped, callback)
    
    rospy.loginfo_once("Hey! gps_utm_logger is up&running, listening to %s", input_topic)

    rospy.spin()

if __name__ == '__main__':
    listener()

    
