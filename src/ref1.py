#!/usr/bin/env python3

# 무한루프가 rough하게 재생되는 예제코드
import rospy
import rosbag
from std_msgs.msg import String
import os

def play_bag(bag_path):
    with rosbag.Bag(bag_path) as bag:
        rospy.loginfo("Playing rosbag...")
        for topic, msg, t in bag.read_messages(topics=['chatter']):
            if rospy.is_shutdown():
                break
            pub.publish(msg)
            rospy.sleep(2.0)
    rospy.loginfo("Rosbag playback finished.")

if __name__ == '__main__':
    rospy.init_node('bag_play_node', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # Define the bag file path
    bag_dir = os.path.expanduser('~/catkin_ws/src/rosbag_test_pkg/bags')
    bag_path = os.path.join(bag_dir, 'test.bag')

    if not os.path.exists(bag_path):
        rospy.logerr("Bag file not found: %s", bag_path)
        exit(1)

    while not rospy.is_shutdown():
        play_bag(bag_path)
        rospy.sleep(1.0)  # Optional: Add a small delay between replays to avoid rapid looping
