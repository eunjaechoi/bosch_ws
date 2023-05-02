import rospy
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointField, PointCloud2

import json
import glob
import numpy as np

def lidar_data(filename, frame_id, dt):


    # read binary data
    scan = (np.fromfile(filename, dtype=np.float32)).reshape(-1, 4)

    # create header
    header = Header()
    header.frame_id = 'world'
    # header.stamp = dt

    # fill pcl msg
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1)]
    msg = pcl2.create_cloud(header, fields, scan)

    return msg


if __name__ == '__main__':

    rospy.init_node("gt_viz", anonymous=True)

    pub_fr_view_ldr_upper = rospy.Publisher("/fr_view_ldr_upper/point_cloud2", PointCloud2, queue_size=10)
    pub_ar_view_ldr = rospy.Publisher("/ar_view_ldr/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_left = rospy.Publisher("/near_view_ldr_left/point_cloud2", PointCloud2, queue_size=10)
    pub_rr_view_ldr = rospy.Publisher("/rr_view_ldr/point_cloud2", PointCloud2, queue_size=10)
    pub_fr_view_ldr_lower = rospy.Publisher("/fr_view_ldr_lower/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_rear = rospy.Publisher("/near_view_ldr_rear/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_right = rospy.Publisher("/near_view_ldr_right/point_cloud2", PointCloud2, queue_size=10)

    path = '/media/nika/storage/01_PoC/01_Bosch/'
    pc_path = f'{path}/test/*.bin'
    pc_path_list = glob.glob(pc_path)

    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # 세그멘테이션 구분된 포인트클라우드 Publish
        for pc_path in pc_path_list:
            dt = 0
            lidar_name = pc_path.split('/')[-1].split('_')[1].split('.')[0]

            if lidar_name == 'FR-View-LDR-Upper':
                lidar_msg = lidar_data(pc_path, 'fr_view_ldr_upper', dt)
                pub_fr_view_ldr_upper.publish(lidar_msg)
            elif lidar_name == 'AR-View-LDR':
                lidar_msg = lidar_data(pc_path, 'ar_view_ldr', dt)
                pub_ar_view_ldr.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Left':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_left', dt)
                pub_near_view_ldr_left.publish(lidar_msg)
            elif lidar_name == 'RR-View-LDR':
                lidar_msg = lidar_data(pc_path, 'rr_view_ldr', dt)
                pub_rr_view_ldr.publish(lidar_msg)
            elif lidar_name == 'FR-View-LDR-Lower':
                lidar_msg = lidar_data(pc_path, 'fr_view_ldr_lower', dt)
                pub_fr_view_ldr_lower.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Rear':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_rear', dt)
                pub_near_view_ldr_rear.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Right':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_right', dt)
                pub_near_view_ldr_right.publish(lidar_msg)

        loop_rate.sleep()