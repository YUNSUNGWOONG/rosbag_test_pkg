#!/usr/bin/env python3

""" 
`/ouster/points` 토픽으로부터 정보를 받아와 차선만 인식할 수 있도록 필터링 하는 예제코드(pclpy패키지 사용)
""" 

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from pclpy import pcl

def point_cloud_callback(msg):
    # Convert ROS PointCloud2 message to numpy array
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    cloud_points = np.array(cloud_points, dtype=np.float32)

    # Create a PCL point cloud from the numpy array
    pcl_cloud = pcl.PointCloud.PointXYZ()
    pcl_cloud.from_array(cloud_points)

    # Apply a simple height filter to isolate the lane points
    passthrough = pcl.filters.PassThrough.PointXYZ()
    passthrough.set_input_cloud(pcl_cloud)
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.0, 0.1)
    lane_points = pcl.PointCloud.PointXYZ()
    passthrough.filter(lane_points)

    rospy.loginfo("Number of lane points detected: %d", lane_points.size)

def main():
    rospy.init_node('lane_detection_node', anonymous=True)
    
    # Subscribe to the /ouster/points topic
    rospy.Subscriber('/ouster/points', PointCloud2, point_cloud_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()