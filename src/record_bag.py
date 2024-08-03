#!/usr/bin/env python3

import rospy
import random
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import rosbag
from std_msgs.msg import String
import os

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    bag.write('chatter', data)

if __name__ == '__main__':
    rospy.init_node('bag_record_node', anonymous=True)
    
    # Ensure the directory exists
    bag_dir = os.path.expanduser('~/catkin_ws/src/rosbag_test_pkg/bags')
    if not os.path.exists(bag_dir):
        os.makedirs(bag_dir)

    # Define the bag file path
    bag_path = os.path.join(bag_dir, 'test.bag')

    pub = rospy.Publisher("pose", PoseStamped, queue_size=1)
    rospy.Subscriber("chatter", String, callback)

    bag = rosbag.Bag(bag_path, 'w')
    rospy.loginfo("Recording rosbag...")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bag.close()
        rospy.loginfo("Rosbag recording finished.")
