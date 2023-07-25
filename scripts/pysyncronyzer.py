import argparse
import rospy
import message_filters
from std_msgs.msg import Int32, Float32

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

km_h=30
slop=0.01
window=50
lidar_topic='/velodyne_points'
image_topic='/camera/image_raw'

def callback(lidar, image):
    rospy.loginfo_once('First message received. No other message will be printed. You can change log level to debug to see some more info.')
    diff=(abs(lidar.header.stamp-image.header.stamp).to_nsec()/1000000000)
    rospy.logdebug("lidar %d.%d\t image: %d.%d\t diff: %.9fs\t distance %d km/h: %fm", 
                  lidar.header.stamp.secs, 
                  lidar.header.stamp.nsecs, 
                  image.header.stamp.secs,
                  image.header.stamp.nsecs, 
                  diff, 
                  km_h, 
                  diff*km_h/3.6)
    syncpub_lidar.publish(lidar)
    syncpub_image.publish(image)
    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--kmh', dest='km_h', type=int, default=30, help='set the speed used to evaluate displacement')
    parser.add_argument('--slop', dest='slop', type=float, default=0.01, help='windows in seconds for syncronization')
    parser.add_argument('--window', dest='window', type=int, default=50, help='window for message syncronization')
    parser.add_argument('--lidar_topic', dest='lidar_topic', type=str, default='/velodyne_points', help='velodyne point cloud topic')
    parser.add_argument('--image_topic', dest='image_topic', type=str, default='/camera/image_raw', help='velodyne point cloud topic')
    
    args, unknown = parser.parse_known_args()
    km_h = args.km_h
    slop = args.slop
    window = args.window
    lidar_topic = args.lidar_topic
    image_topic = args.image_topic
    
    rospy.init_node('pysyncronizer', anonymous=True)  

    unsync_lidar = message_filters.Subscriber(lidar_topic, PointCloud2)
    unsync_image = message_filters.Subscriber(image_topic, Image)

    syncpub_lidar = rospy.Publisher('sync_lidar', PointCloud2, queue_size=1)
    syncpub_image = rospy.Publisher('sync_image', Image, queue_size=1)


    ts = message_filters.ApproximateTimeSynchronizer([unsync_lidar, unsync_image], 50, 0.01, allow_headerless=False)
    ts.registerCallback(callback)

    rospy.loginfo_once("Hey! pysyncronizer is up&running, listening to %s and %s. Press ctrl+c to kill this node.", lidar_topic, image_topic)
    rospy.loginfo_once("Waiting for messages...")
    rospy.spin()

    print("\nGoodbye!")
