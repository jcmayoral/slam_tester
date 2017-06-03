import rospy
from sensor_msgs.msg import PointCloud2 as PC2

class Republisher:

    def __init__(self):
        rospy.init_node('slam_republisher', anonymous=True)
        rospy.Subscriber("/camera/depth_registered/points",PC2, self.getDepthCB)
        self.depth_Publisher = rospy.Publisher("/points2", PC2)
        
        print "Republisher Object Initialized"

    def getDepthCB(self,msg):
        self.depth_Publisher.publish(msg)
    	
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

