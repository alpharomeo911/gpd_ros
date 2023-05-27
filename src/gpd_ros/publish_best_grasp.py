#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose
import numpy as np
from gpd_ros.msg import GraspConfig, GraspConfigList
from scipy.spatial.transform import Rotation as R
import tf.transformations as tr



class publish_best_grasp(object):
    def __init__(self):
        self.pose = Pose()
        self.grasp_config = None
        self.orientation = np.array([])
        self.rate = rospy.Rate(10)
        self.best_grasp_publisher = rospy.Publisher("/best_grasp_pc", Pose, queue_size=10)
        self.grasp_array_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.callback, queue_size=10)
    

    def callback(self, msg):
        rospy.loginfo("[+] Message Received")
        self.grasp_config = msg.grasps[0]
        self.pose.position.x = self.grasp_config.position.x
        self.pose.position.y = self.grasp_config.position.y
        self.pose.position.z = self.grasp_config.position.z
        self.orientation = np.array([[self.grasp_config.approach.x, self.grasp_config.approach.y, self.grasp_config.approach.z, 0],
                                     [self.grasp_config.binormal.x, self.grasp_config.binormal.y, self.grasp_config.binormal.z, 0],
                                     [self.grasp_config.axis.x, self.grasp_config.axis.y, self.grasp_config.axis.z, 0],
                                     [0, 0, 0, 1]])@tr.rotation_matrix(np.math.pi/2, (0,1,0))@tr.rotation_matrix(-np.math.pi/2, (1,0,0))
        self.orientation = tr.quaternion_from_matrix(self.orientation)
        self.pose.orientation.x = self.orientation[0]
        self.pose.orientation.y = self.orientation[1]
        self.pose.orientation.z = self.orientation[2]
        self.pose.orientation.w = self.orientation[3]
        self.best_grasp_publisher.publish(self.pose)


    
    def start(self):
        rospy.loginfo("[+] Grasp node Publisher started")
        rospy.spin()
        while not rospy.is_shutdown():
            self.rate.sleep()
    



if __name__ == '__main__':
    rospy.init_node("best_grasp", anonymous=True)
    grasper = publish_best_grasp()
    grasper.start()