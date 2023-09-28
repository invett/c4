#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

counter = 0

def callback(data):
    global counter    
    filename=str(counter)
    filename=str(data.header.stamp)+'_'+(filename.rjust(10, '0')+'.txt')    
    f = open(filename, "w")    
    f.write(str(data.latitude)+','+str(data.longitude)+','+str(data.altitude))
    f.close()
    
    rospy.loginfo("Filename: %s \t latitude %s \t longitude %s \t altitude%s", filename, data.latitude, data.longitude, data.altitude)
    
    counter = counter + 1

    
def listener():

    rospy.init_node('gps_utm_logger', anonymous=True)
    
    input_topic = "/tosave_gps_latlon"   #published by pysyncronizer2

    rospy.Subscriber(input_topic, NavSatFix, callback)
    
    rospy.loginfo_once("Hey! gps_utm_logger is up&running, listening to %s", input_topic)

    rospy.spin()

if __name__ == '__main__':
    listener()

    
