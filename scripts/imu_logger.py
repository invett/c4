#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

counter = 0

def callback(data):
    global counter    
    filename=str(counter)
    filename=str(data.header.stamp)+'_'+(filename.rjust(10, '0')+'.txt')    
    f = open(filename, "w")    
    f.write(str(data.orientation.x)+','+str(data.orientation.y)+','+str(data.orientation.z)+','+str(data.orientation.w))
    f.close()
    
    rospy.loginfo("Filename: %s \t x: %s \t y: %s \t z: %s \t w: %s", filename, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    
    counter = counter + 1

    
def listener():

    rospy.init_node('imu_logger', anonymous=True)
    
    input_topic = "/tosave_imu"   #published by pysyncronizer2

    rospy.Subscriber(input_topic, Imu, callback)
    
    rospy.loginfo_once("Hey! imu_logger is up&running, listening to %s", input_topic)

    rospy.spin()

if __name__ == '__main__':
    listener()

    
