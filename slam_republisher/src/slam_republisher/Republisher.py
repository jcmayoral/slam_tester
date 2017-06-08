import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped, Quaternion

class Republisher:

    def __init__(self):
        rospy.init_node('slam_republisher', anonymous=True)
        rospy.Subscriber("~/base/twist_mux/command_teleop_keyboard",Twist, self.getTwistCB)
        rospy.Subscriber("~/amcl_pose",PoseWithCovarianceStamped, self.getPoseCB)
        self.imu_Publisher = rospy.Publisher("/Imu", Imu)
        self.freq = 10
        self.count = 0
        self.is_Pose_received = False
        self.is_Twist_received = False
        self.is_shutdown = False
        print "Republisher Node Initialized"

    def getPoseCB(self,msg):
        self.orientation = Quaternion()
        self.orientation = msg.pose.pose.orientation
        self.is_Pose_received = True

    def getTwistCB(self,msg):
        self.twist = Twist()
        self.twist = msg
        self.is_Twist_received = True

    def publishImuMsg(self):
        #This code do not consider covariance
        while not self.is_Twist_received and not self.is_Twist_received:
            return
        tmp = Imu()
        tmp.header.seq = self.count
        tmp.header.frame_id = "base_link"
        tmp.orientation = self.orientation
        tmp.angular_velocity.z = self.twist.angular.z
        tmp.linear_acceleration.x = self.twist.linear.x/self.freq
        tmp.linear_acceleration.y = self.twist.linear.y/self.freq
        self.imu_Publisher.publish(tmp)
        self.count = self.count + 1
        self.is_Pose_received = False
        self.is_Twist_received = False

    def shutdown(self):
        self.is_shutdown = True
        print("Node Down")

    def run(self, frequency = 0):

        if frequency is not 0:
            self.freq = frequency

        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(self.freq)

        while not self.is_shutdown:
            self.publishImuMsg()
        print ("Node Shutting down")
