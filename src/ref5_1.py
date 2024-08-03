#!/usr/bin/env python3

""" 
`/ouster/points` 토픽으로부터 정보를 받아와 차선만 인식할 수 있도록 필터링 하는 예제코드(pcl패키지 사용)
""" 

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import pcl_visualization
from pcl_ros import point_cloud2

def point_cloud_callback(msg):
    # Convert ROS PointCloud2 message to PCL PointCloud
    cloud = point_cloud2.pointcloud2_to_xyz_array(msg)

    # Create a PCL point cloud from the numpy array
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_array(cloud.astype(np.float32))

    # Apply a simple height filter to isolate the lane points
    # Assuming the lane points are located at a certain height range
    passthrough = pcl_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.0, 0.1)
    lane_points = passthrough.filter()

    rospy.loginfo("Number of lane points detected: %d", lane_points.size)

    # Publish or visualize the lane points if needed
    # For visualization using PCL
    visual = pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(lane_points)
    visual.Spin()

def main():
    rospy.init_node('lane_detection_node', anonymous=True)
    
    # Subscribe to the /ouster/points topic
    rospy.Subscriber('/ouster/points', PointCloud2, point_cloud_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

