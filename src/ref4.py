#!/usr/bin/env python3

""" 
`/ouster/points` 토픽으로부터 정보를 받아와 차선만 인식할 수 있도록 필터링 하는 예제코드(pcl패키지 사용안함)
""" 

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def point_cloud_callback(msg):
    # Convert ROS PointCloud2 message to PCL PointCloud
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    cloud_points = np.array(cloud_points)

    # Apply a simple height filter to isolate the lane points
    # Assuming the lane points are located at a certain height range
    z_min = 0.0  # minimum height of the lane
    z_max = 0.1  # maximum height of the lane

    lane_points = cloud_points[(cloud_points[:, 2] >= z_min) & (cloud_points[:, 2] <= z_max)]

    # Further processing can be done to detect lanes, such as clustering or RANSAC

    rospy.loginfo("Number of lane points detected: %d", lane_points.shape[0])

    # Publish or visualize the lane points if needed

def main():
    rospy.init_node('lane_detection_node', anonymous=True)
    
    # Subscribe to the /ouster/points topic
    rospy.Subscriber('/ouster/points', PointCloud2, point_cloud_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

