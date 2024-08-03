#!/usr/bin/env python3

""" 
`/ouster/points` 토픽으로부터 정보를 받아와 차선만 인식할 수 있도록 필터링 하는 예제코드(pcl패키지 사용)
""" 

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pcl

def point_cloud_callback(msg):
    # Convert ROS PointCloud2 message to PCL PointCloud
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    cloud_points = np.array(cloud_points, dtype=np.float32)

    # Create a PCL point cloud from the numpy array
    pcl_cloud = pcl.PointCloud(cloud_points)

    # Apply a simple height filter to isolate the lane points
    passthrough = pcl_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.0, 0.1)
    lane_points = passthrough.filter()

    rospy.loginfo("Number of lane points detected: %d", lane_points.size)

def main():
    rospy.init_node('lane_detection_node', anonymous=True)
    
    # Subscribe to the /ouster/points topic
    rospy.Subscriber('/ouster/points', PointCloud2, point_cloud_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()