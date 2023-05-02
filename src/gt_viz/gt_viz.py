import rospy
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointField, PointCloud2, Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import tf

import cv2
import json
import glob
import math

from projection import *


def get_calib_param(calib_data):
    calib_param = dict()
    for calib in calib_data:
        if 'cam_name' in calib.keys():
            cam_name = calib['cam_name']
            calib_param[cam_name]=dict()
            calib_param[cam_name]['model_type'] = calib['model_type']
            calib_param[cam_name]['intrinsic_matrix'] = np.array(
                                                        [[calib['f_x'],  0,           calib['c_x']],
                                                        [0,            calib['f_y'], calib['c_y']],
                                                        [0,            0,           1]])

            calib_param[cam_name]['extrinsic_matrix'] = np.array(calib['world_to_cam']).reshape((4,4))
            calib_param[cam_name]['translation_matrix'] = np.array(calib_param[cam_name]['extrinsic_matrix'][:3, 3])
            calib_param[cam_name]['rotation_matrix'] = calib_param[cam_name]['extrinsic_matrix'][:3, :3]

            projection_matrix = np.vstack((calib_param[cam_name]['intrinsic_matrix'], [0, 0, 0]))
            projection_matrix = np.hstack((projection_matrix, [[0], [0], [0], [1]]))
            # projection_matrix = np.dot(projection_matrix, calib_param[cam_name]['extrinsic_matrix'])
            calib_param[cam_name]['projection_matrix'] = projection_matrix
            
            if 'radial_distortion' in calib.keys():
                calib_param[cam_name]['radial_distortion'] = calib['radial_distortion']
            else:
                calib_param[cam_name]['radial_distortion'] = [0.0, 0.0]
            if 'tangential_distortion' in calib.keys():
                calib_param[cam_name]['tangential_distortion'] = calib['tangential_distortion']
            else:
                calib_param[cam_name]['tangential_distortion'] = [0.0, 0.0]
            if 'mirror_parameter' in calib.keys():
                calib_param[cam_name]['mirror_parameter'] = calib['mirror_parameter']
            else:
                calib_param[cam_name]['mirror_parameter'] = 0
            
            calib_param[cam_name]['distortion_coefficients'] = np.array([calib['radial_distortion'][0], calib['radial_distortion'][1], calib['tangential_distortion'][0], calib['tangential_distortion'][1]])
    return calib_param

def get_gt_img(img_path, calib_data, label_path):

    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    img_h,  img_w = img.shape[:2]

    calib_data = json.load(open(calib_path))
    calib_param = get_calib_param(calib_data)


    cam_name = img_path.split('/')[-1].split('_')[1].split('.')[0]

    if cam_name == 'FR-View-CMR-Wide':
        calib_cam_name = 'front_120'
    elif cam_name == 'FR-View-CMR-Narrow':
        calib_cam_name = 'front_60'
    elif cam_name == 'RR-Left-View-CMR-Narrow':
        calib_cam_name = 'left'
    elif cam_name == 'RR-Right-View-CMR-Narrow':
        calib_cam_name = 'right'
    elif cam_name == 'RR-View-CMR-Wide':
        calib_cam_name = 'rear'
    elif cam_name == 'Near-View-CMR-Front':
        calib_cam_name = 'svm_front'
    elif cam_name == 'Near-View-CMR-Left':
        calib_cam_name = 'svm_left'
    elif cam_name == 'Near-View-CMR-Rear':
        calib_cam_name = 'svm_rear'
    elif cam_name == 'Near-View-CMR-Right':
        calib_cam_name = 'svm_right'

    label_data = json.load(open(label_path))
    labels = label_data['annotations']

    bbox_img_list = []

    for label in labels:
        if label['type'] == 'cuboid':
            # front_center camera projection
            cam_view = calib_cam_name
            bbox_ego = label_to_bbox(label)
            bbox_cam, depth = ego_to_camera_coord(bbox_ego, calib_param[cam_view], label['position'])
            bbox_img = camera_to_image_coord(bbox_cam, calib_param[cam_view])

            if depth < 0.1:
                continue

            if not 'label' in label.keys():
                print('label이 없습니다.')
                continue
            
            if 'trackId' in label.keys():
                bbox_img_list.append([bbox_img, label['label'], label['trackId']])
            elif 'track_id' in label.keys():
                bbox_img_list.append([bbox_img, label['label'], label['track_id']])
            else:
                print('track id가 없습니다.')
                continue

    # 이미지 내부에 큐보이드의 꼭지점이 하나라도 포함되는지 확인
    bbox_img_filter_list = []
    for bbox_img in bbox_img_list:
        bbox_img_check = False
        for i in range(8):
            if bbox_img[0][0][i] >= 0 and bbox_img[0][0][i] <= img_w:
                if bbox_img[0][1][i] >= 0 and bbox_img[0][1][i] <= img_h:
                    bbox_img_check = True
        if bbox_img_check == True:
            bbox_img_filter_list.append(bbox_img)

    # 큐보이드가 뒤집혔는지 확인
    bbox_img_filter_list2 = []
    for bbox_img in bbox_img_filter_list:
        if (bbox_img[0][1][0] < bbox_img[0][1][4]) or (bbox_img[0][1][1] < bbox_img[0][1][5]) or (bbox_img[0][1][2] < bbox_img[0][1][7]) or (bbox_img[0][1][3] < bbox_img[0][1][6]):
            continue
        bbox_img_filter_list2.append(bbox_img)

    # visualize
    if len(bbox_img_filter_list2) > 0:

        for bbox_img in bbox_img_filter_list2:
            
            text_color = (255, 255, 255)

            if bbox_img[1] == 'car':
                bbox_img[1] = 'CAR'
                line_color = (5, 255, 255)
                box_color = (255, 255, 5)
            elif bbox_img[1] == 'bus':
                bbox_img[1] = 'BUS'
                line_color = (5, 65, 106)
                box_color = (106, 65, 5)
            elif bbox_img[1] == 'adult':
                bbox_img[1] = 'ADT'
                line_color = (105, 155, 45)
                box_color = (45, 155, 105)
            elif bbox_img[1] == 'truck':
                bbox_img[1] = 'TK'
                line_color = (5, 5, 75)
                box_color = (75, 5, 5)
            elif bbox_img[1] == 'motorcyclist':
                bbox_img[1] = 'MC_RID'
                line_color = (5, 5, 75)
                box_color = (75, 5, 5)
            elif bbox_img[1] == 'personal_mobility' or bbox_img[1] == 'personal_mobility_rider':
                bbox_img[1] = 'PM_RID'
                line_color = (105, 155, 45)
                box_color = (45, 155, 105)
            elif bbox_img[1] == 'unknown-vehicle':
                bbox_img[1] = 'UNK_VEH'
                line_color = (1, 1, 1)
                box_color = (1, 1, 1)
            else:
                bbox_img[1] = 'UNK_VEH'
                line_color = (1, 1, 1)
                box_color = (1, 1, 1)

            # 큐보이드 선 그리기
            draw_projected_bbox(img, bbox_img[0], colors=(line_color, line_color, line_color))
            trans_bbox_img = np.transpose(bbox_img[0])
            # for i in range(len(trans_bbox_img)):
            #     cv2.putText(img, str(i), (int(trans_bbox_img[i][0]), int(trans_bbox_img[i][1]) - 6), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
            text = bbox_img[1] + ': ' + str(bbox_img[2])
            text_size = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=0.7, thickness=1)
            text_size_width, text_size_height = text_size[0]

            # 큐보이드 박스 그리기
            cv2.rectangle(img, (int(bbox_img[0][0][5]), int(bbox_img[0][1][5] - 17)), (int(bbox_img[0][0][5]) + text_size_width - 20, int(bbox_img[0][1][5] - 17) + text_size_height), box_color, -1)
            cv2.putText(img, text, (int(bbox_img[0][0][5]), int(bbox_img[0][1][5] - 6)), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7, text_color, 1, cv2.LINE_AA)

    return img

def lidar_data(filename, frame_id, dt, labels):


    # read binary data
    scan = (np.fromfile(filename, dtype=np.float32)).reshape(-1, 4)

    max_z = 6.0
    inrange = np.where((scan[:, 2] < max_z))
    scan = scan[inrange[0]]

    # for label in labels:
    #     if label['type'] == 'cuboid':

    #         x, y, z = label["position"]
    #         l, w, h = label["geometry"]
    #         roll, pitch, yaw = label["rotation"]

    #         x_corners = l / 2 * np.array([-1, 1])
    #         y_corners = w / 2 * np.array([-1, 1])
    #         z_corners = h / 2 * np.array([-1, 1])

    #         corners = np.vstack((x_corners, y_corners, z_corners))
    #         corners = np.reshape(corners.T, (-1, 3))

    #         corners = np.array([Quaternion(axis=[0, 0, 1], radians=yaw).rotate(corner) for corner in corners])
    #         corners = np.array([Quaternion(axis=[0, 1, 0], radians=pitch).rotate(corner) for corner in corners])
    #         corners = np.array([Quaternion(axis=[1, 0, 0], radians=roll).rotate(corner) for corner in corners])

    #         corners[:, 0] = corners[:, 0] + x
    #         corners[:, 1] = corners[:, 1] + y
    #         corners[:, 2] = corners[:, 2] + z

    #         inrange = np.where((scan[:, 0] > corners[0][0]) &
    #                         (scan[:, 0] < corners[1][0]) &
    #                         (scan[:, 1] > corners[0][1]) &
    #                         (scan[:, 1] < corners[1][1]) &
    #                         (scan[:, 2] > corners[0][2]) &
    #                         (scan[:, 2] < corners[1][2]))

    #         if inrange[0].size == 0:
    #             continue

    #         label_name = label['label']

    #         max_color = 255
    #         intensity = 0.0
    #         if label_name == 'car':
    #             intensity = 5/max_color + 255/max_color + 255/max_color
    #             intensity = 255
    #         elif label_name == 'bus':
    #             intensity = 5/max_color + 65/max_color + 106/max_color
    #             intensity = 255
    #         elif label_name == 'adult':
    #             intensity = 105/max_color + 155/max_color + 45/max_color
    #             intensity = 255
    #         elif label_name == 'truck':
    #             intensity = 5/max_color + 5/max_color + 75/max_color
    #             intensity = 255
    #         elif label_name == 'motorcyclist':
    #             intensity = 5/max_color + 5/max_color + 75/max_color
    #         elif label_name == 'personal_mobility' or label_name == 'personal_mobility_rider':
    #             intensity = 105/max_color + 155/max_color + 455/max_color
    #             intensity = 255
    #         elif label_name == 'unknown-vehicle':
    #             intensity = 1/max_color + 1/max_color + 1/max_color
    #             intensity = 255
    #         else:
    #             intensity = 1/max_color + 1/max_color + 1/max_color
    #             intensity = 255

    #         for i in inrange[0]:
    #             scan[i][3] = intensity

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

def get_cuboid_marker(label):

    label_name = label['label']

    x, y, z = label["position"]
    l, w, h = label["geometry"]
    roll, pitch, yaw = label["rotation"]
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    marker = Marker()

    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = l
    marker.scale.y = w
    marker.scale.z = h


    max_color = 255
    if label_name == 'car':
        marker.color.r = 5/max_color
        marker.color.g = 255/max_color
        marker.color.b = 255/max_color
    elif label_name == 'bus':
        marker.color.r = 5/max_color
        marker.color.g = 65/max_color
        marker.color.b = 106/max_color
    elif label_name == 'adult':
        marker.color.r = 105/max_color
        marker.color.g = 155/max_color
        marker.color.b = 45/max_color
    elif label_name == 'truck':
        marker.color.r = 5/max_color
        marker.color.g = 5/max_color
        marker.color.b = 75/max_color
    elif label_name == 'motorcyclist':
        marker.color.r = 5/max_color
        marker.color.g = 5/max_color
        marker.color.b = 75/max_color
    elif label_name == 'personal_mobility' or label_name == 'personal_mobility_rider':
        marker.color.r = 105/max_color
        marker.color.g = 155/max_color
        marker.color.b = 45/max_color
    elif label_name == 'unknown-vehicle':
        marker.color.r = 1/max_color
        marker.color.g = 1/max_color
        marker.color.b = 1/max_color
    else:
        marker.color.r = 1/max_color
        marker.color.g = 1/max_color
        marker.color.b = 1/max_color
    marker.color.a = 0.5

    # Set the color

    # Set the pose of the marker
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    return marker

def get_text_marker(label):

    label_name = label['label']

    x, y, z = label["position"]
    l, w, h = label["geometry"]
    roll, pitch, yaw = label["rotation"]
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    marker = Marker()

    marker.header.frame_id = "world"
    marker.action = marker.ADD
    marker.type = Marker.TEXT_VIEW_FACING
    marker.scale.x = 0.7
    marker.scale.y = 0.7
    marker.scale.z = 0.7
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z + h/2 + 0.3

    marker.id = 0

    dist = round(math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2)), 2)
    marker.text = str(dist)

    return marker

def get_line_maker(label):

    x, y, z = label["position"]

    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = 0  # self.subdivisions
    marker.action = Marker.ADD
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(x, y, z))

    return marker


if __name__ == '__main__':

    rospy.init_node("gt_viz", anonymous=True)

    bridge = CvBridge()
    front_img_pub = rospy.Publisher('/front_img_gt_viz', Image, queue_size=10)
    left_img_pub = rospy.Publisher('/left_img_gt_viz', Image, queue_size=10)
    right_img_pub = rospy.Publisher('/right_img_gt_viz', Image, queue_size=10)
    rear_img_pub = rospy.Publisher('/rear_img_gt_viz', Image, queue_size=10)

    pub_fr_view_ldr_upper = rospy.Publisher("/fr_view_ldr_upper/point_cloud2", PointCloud2, queue_size=10)
    pub_ar_view_ldr = rospy.Publisher("/ar_view_ldr/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_left = rospy.Publisher("/near_view_ldr_left/point_cloud2", PointCloud2, queue_size=10)
    pub_rr_view_ldr = rospy.Publisher("/rr_view_ldr/point_cloud2", PointCloud2, queue_size=10)
    pub_fr_view_ldr_lower = rospy.Publisher("/fr_view_ldr_lower/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_rear = rospy.Publisher("/near_view_ldr_rear/point_cloud2", PointCloud2, queue_size=10)
    pub_near_view_ldr_right = rospy.Publisher("/near_view_ldr_right/point_cloud2", PointCloud2, queue_size=10)

    pub_cuboid = rospy.Publisher("/cuboid", MarkerArray, queue_size = 10)
    pub_text = rospy.Publisher("/text", MarkerArray, queue_size = 10)
    pub_line = rospy.Publisher("/line", MarkerArray, queue_size = 10)

    path = '/media/nika/storage/01_PoC/01_Bosch/viz_data/'
    img_path = f'{path}01_img/*.png'
    pc_path = f'{path}02_pcd/*.bin'
    label_path = f'{path}03_gt/1659923557471_Merged-LDR-AR+Near_Cuboid_GT.json'
    calib_path = f'{path}04_cal/220708-220824_calibration.json'


    img_path_list = glob.glob(img_path)
    pc_path_list = glob.glob(pc_path)
    label_data = json.load(open(label_path))
    labels = label_data['annotations']

    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker_array_cuboid = MarkerArray()
        marker_array_text = MarkerArray()
        marker_array_line = MarkerArray()

        # 큐보이드 투영 이미지 Publish
        for img_path in img_path_list:
            cam_name = img_path.split('/')[-1].split('_')[1].split('.')[0]
            img = get_gt_img(img_path, calib_path, label_path)

            if cam_name == 'FR-View-CMR-Wide':
                front_img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            elif cam_name == 'RR-Left-View-CMR-Narrow':
                left_img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            elif cam_name == 'RR-Right-View-CMR-Narrow':
                right_img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            elif cam_name == 'RR-View-CMR-Wide':
                rear_img_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

        # 세그멘테이션 구분된 포인트클라우드 Publish
        for pc_path in pc_path_list:
            dt = 0
            lidar_name = pc_path.split('/')[-1].split('_')[1].split('.')[0]

            if lidar_name == 'FR-View-LDR-Upper':
                lidar_msg = lidar_data(pc_path, 'fr_view_ldr_upper', dt, labels)
                pub_fr_view_ldr_upper.publish(lidar_msg)
            elif lidar_name == 'AR-View-LDR':
                lidar_msg = lidar_data(pc_path, 'ar_view_ldr', dt, labels)
                pub_ar_view_ldr.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Left':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_left', dt, labels)
                pub_near_view_ldr_left.publish(lidar_msg)
            elif lidar_name == 'RR-View-LDR':
                lidar_msg = lidar_data(pc_path, 'rr_view_ldr', dt, labels)
                pub_rr_view_ldr.publish(lidar_msg)
            elif lidar_name == 'FR-View-LDR-Lower':
                lidar_msg = lidar_data(pc_path, 'fr_view_ldr_lower', dt, labels)
                pub_fr_view_ldr_lower.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Rear':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_rear', dt, labels)
                pub_near_view_ldr_rear.publish(lidar_msg)
            elif lidar_name == 'Near-View-LDR-Right':
                lidar_msg = lidar_data(pc_path, 'near_view_ldr_right', dt, labels)
                pub_near_view_ldr_right.publish(lidar_msg)
        

        # Cuboid Publish
        bbox_img_list = []
        depth_list = []
        for label in labels:
            if label['type'] == 'cuboid':

                cuboid_marker = get_cuboid_marker(label)
                text_marker = get_text_marker(label)
                line_marker = get_line_maker(label)
                marker_array_cuboid.markers.append(cuboid_marker)
                marker_array_text.markers.append(text_marker)
                marker_array_line.markers.append(line_marker)
        id = 0
        for m in marker_array_cuboid.markers:
            m.id = id
            id += 1
        id = 0
        for m in marker_array_text.markers:
            m.id = id
            id += 1
        id = 0
        for m in marker_array_line.markers:
            m.id = id
            id += 1
            
        pub_cuboid.publish(marker_array_cuboid)
        pub_text.publish(marker_array_text)
        pub_line.publish(marker_array_line)

        loop_rate.sleep()