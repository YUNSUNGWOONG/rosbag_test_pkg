#!/usr/bin/env python3

""" 
`visual_rviz_node`노드가 `/ouster/points`토픽으로부터 정보를 받을 수 있도록 
해당 토픽을 구독하는 예제코드
""" 

import rospy
from sensor_msgs.msg import PointCloud2  # Import the PointCloud2 message type

def point_cloud_callback(data):
    rospy.loginfo("Received point cloud data")
    # Here you can add the code to process the point cloud data

if __name__ == '__main__':
    rospy.init_node('visual_rviz_node', anonymous=True)
    
    # Subscribe to the /ouster/points topic
    rospy.Subscriber('/ouster/points', PointCloud2, point_cloud_callback)
    
    rospy.spin()

