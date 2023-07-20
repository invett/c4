import rospy
import message_filters
from std_msgs.msg import Int32, Float32

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

km_h=30

def callback(lidar, image):
    rospy.loginfo_once("Hey! pysyncronizer is up&running, loggin in debug level. Press ctrl+c to kill this node.")
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
    
rospy.init_node('pysyncronizer', anonymous=True)  

unsync_lidar = message_filters.Subscriber('/velodyne_points', PointCloud2)
unsync_image = message_filters.Subscriber('/camera/image_raw', Image)

syncpub_lidar = rospy.Publisher('sync_lidar', PointCloud2, queue_size=1)
syncpub_image = rospy.Publisher('sync_image', Image, queue_size=1)


ts = message_filters.ApproximateTimeSynchronizer([unsync_lidar, unsync_image], 50, 0.01, allow_headerless=False)
ts.registerCallback(callback)

rospy.spin()

print("\nGoodbye!")
