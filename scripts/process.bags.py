#!/usr/bin/env python

"""

    This script is used to prepare dataset folders from .bag
    
    

"""


import argparse
import rospy
import message_filters
from std_msgs.msg import Int32, Float32

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

import pypcd

km_h=30
slop=0.01
window=50

lidar_topic='/sync_lidar'
image_topic='/sync_image'

counter = 0

def callback(lidar, image):
    
    global counter    
    filename=str(counter)
    filename=str(image.header.stamp)+'_'+(filename.rjust(10, '0')+'.png')    
    
    rospy.loginfo_once('First message received. Other message will be printed periodically (10s seconds).')
        
    # Convert the compressed image message to a BGR image
    bridge = CvBridge()
    compressed_image = bridge.compressed_imgmsg_to_cv2(image, desired_encoding="bgr8")
    debayered_image = cv2.cvtColor(compressed_image, cv2.COLOR_BAYER_BG2BGR)

    # Save the debayered image as PNG
    cv2.imwrite(filename, debayered_image)
    print(f"Image saved as {filename}")        
    
    
    syncpub_lidar.publish(lidar)
    
    
    counter = counter + 1

    
    
    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
        
    parser.add_argument('--window', dest='window', type=int, default=50, help='window for message syncronization')
    parser.add_argument('--lidar_topic', dest='lidar_topic', type=str, default='/sync_lidar', help='velodyne point cloud topic')
    parser.add_argument('--image_topic', dest='image_topic', type=str, default='/sync_image', help='imagecloud topic')
    
    args, unknown = parser.parse_known_args()
    
    
    window = args.window
    lidar_topic = args.lidar_topic
    image_topic = args.image_topic
    
    rospy.init_node('prepare.bags', anonymous=True)  

    unsync_lidar = message_filters.Subscriber(lidar_topic, PointCloud2)
    unsync_image = message_filters.Subscriber(image_topic, CompressedImage)
    
    syncpub_lidar = rospy.Publisher('save_lidar', PointCloud2, queue_size=10)
    
    ts = message_filters.ApproximateTimeSynchronizer([unsync_lidar, unsync_image], 50, 0.01, allow_headerless=False)
    ts.registerCallback(callback)

    rospy.loginfo_once("Hey! pysyncronizer is up&running, listening to %s and %s. Press ctrl+c to kill this node.", lidar_topic, image_topic)
    rospy.loginfo_once("Waiting for messages...")
    rospy.spin()

    print("\nGoodbye!")
