import cv2
import math
import numpy as np
from pyquaternion import Quaternion

import matplotlib.pyplot as plt

def label_to_bbox(ann):
    """Aimmo 3d cuboid 포맷을 3d bbox 꼭짓점 좌표로 변환"""
    x, y, z = ann["position"]
    l, w, h = ann["geometry"]
    roll, pitch, yaw = ann["rotation"]

    x_corners = l / 2 * np.array([-1, -1, 1, 1, -1, -1, 1, 1])
    y_corners = w / 2 * np.array([-1, 1, 1, -1, -1, 1, 1, -1])
    z_corners = h / 2 * np.array([-1, -1, -1, -1, 1, 1, 1, 1])

    corners = np.vstack((x_corners, y_corners, z_corners))
    corners = np.reshape(corners.T, (-1, 3))

    corners = np.array([Quaternion(axis=[0, 0, 1], radians=yaw).rotate(corner) for corner in corners])

    corners[:, 0] = corners[:, 0] + x
    corners[:, 1] = corners[:, 1] + y
    corners[:, 2] = corners[:, 2] + z

    return corners.T

def ego_to_camera_coord(bbox_ego, calib, center):
    """ego좌표계(lidar좌표계) 상의 3d bbox를 카메라 좌표계로 변환"""
    ex_mat = calib["extrinsic_matrix"]
    lidar_bbox3d = np.concatenate((bbox_ego, np.ones((1, 8))))
    x, y, z = center

    x_cam, y_cam, z_cam, h = np.dot(ex_mat, [x, y, z, 1])
    depth = z_cam

    bbox_cam = np.dot(ex_mat, lidar_bbox3d)[:3]

    return bbox_cam, depth

def project(bbox_ego, calib):
    """ego좌표계(lidar좌표계) 상의 3d bbox를 카메라에 투영"""
    p_mat = calib["projection_matrix"]
    lidar_bbox3d = np.concatenate((bbox_ego, np.ones((1, 8))))

    bbox_cam = np.dot(p_mat, lidar_bbox3d)[:3]
    bbox_cam = bbox_cam / bbox_cam[2]

    return bbox_cam

def camera_to_image_coord(bbox_cam, calib):
    """카메라 좌표계 상의 3d bbox를 2d 이미지 픽셀 좌표계로 변환"""
    in_mat = calib["intrinsic_matrix"]
    img_bbox2d = np.dot(in_mat, bbox_cam)
    img_bbox2d = img_bbox2d / img_bbox2d[2]
    return img_bbox2d[:2]

def world_to_camera_coord_pointcloud(pointcloud, calib):
    """wrold좌표계 상의 pointcloud를 카메라 좌표계로 변환"""
    ex_mat = calib["extrinsic_matrix"]

    lidar_bbox3d = np.concatenate((pointcloud, np.ones((1, len(pointcloud[0])))))
    pointcloud_cam = np.dot(ex_mat, lidar_bbox3d)[:3]

    return pointcloud_cam

def camera_to_distort_image_coord(bbox_cam, calib):
    """카메라 좌표계 상의 3d Bbox를 2D 왜곡 이미지 좌표계로 변환"""
    if calib["model_type"] == "pinhole":
        return camera_to_image_pinhole(bbox_cam, calib)
    elif calib["model_type"] == "fisheye":
        return camera_to_image_fisheye(bbox_cam, calib)
    else:
        raise ValueError("The camera model type must be pinhole or fisheye")


def camera_to_image_pinhole(bbox_cam, calib):
    in_mat = calib["intr_matrix"]

    # 정규 이미지 좌표계로 변환
    bbox_img_p = bbox_cam / bbox_cam[2]

    # 왜곡 계수
    r_sqr = np.power(bbox_img_p[0], 2) + np.power(bbox_img_p[1], 2)
    k1 = calib["radial_distortion"][0]
    k2 = calib["radial_distortion"][1]
    p1 = calib["tangential_distortion"][0]
    p2 = calib["tangential_distortion"][1]

    r2 = 1 + k1 * r_sqr + k2 * r_sqr ** 2
    bbox_d_img_p = bbox_img_p
    bbox_d_img_p[:2] = r2 * bbox_img_p[:2]
    x = bbox_d_img_p[0]
    y = bbox_d_img_p[1]

    x += 2 * p1 * x * y + p2 * (r2 + 2 * x ** 2)
    y += p1 * (r2 + 2 * y ** 2) + (2 * p2 * x * y)

    bbox_d_img_p[0] = x
    bbox_d_img_p[1] = y
    bbox_d_img_p = np.dot(in_mat, bbox_d_img_p)
    bbox_d_img_p = bbox_d_img_p / bbox_d_img_p[2]

    return bbox_d_img_p[:2]


def camera_to_image_fisheye(bbox_cam, calib):
    k1 = calib["radial_distortion"][0]
    k2 = calib["radial_distortion"][1]
    p1 = calib["tangential_distortion"][0]
    p2 = calib["tangential_distortion"][1]
    fx = calib["intrinsic_matrix"][0, 0]
    fy = calib["intrinsic_matrix"][1, 1]
    cx = calib["intrinsic_matrix"][0, 2]
    cy = calib["intrinsic_matrix"][1, 2]
    m = calib["mirror_parameter"]

    bbox_cam = bbox_cam.T
    norm = np.linalg.norm(bbox_cam, axis=1)

    x = bbox_cam[:, 0] / norm 
    y = bbox_cam[:, 1] / norm
    z = bbox_cam[:, 2] / norm

    x /= z + m 
    y /= z + m 

    r2 = x * x + y * y
    x *= 1 + k1 * r2 + k2 * r2 * r2
    y *= 1 + k1 * r2 + k2 * r2 * r2
    
    x = fx * x + cx
    y = fy * y + cy
    
    bbox_d_img_p = np.vstack((x, y, norm * bbox_cam[:, 2] / np.abs(bbox_cam[:, 2])))
    
    return bbox_d_img_p


def draw_projected_bbox(
    image,
    corners,
    colors=((255, 51, 153), (255, 51, 153), (255, 51, 153)),
    linewidth=2,
):
    """이미지 위에 투영된 3d bbox 그리기"""
    def draw_rect(image, selected_corners, color, linewidth):
        prev = selected_corners[-1]
        for corner in selected_corners:
            cv2.line(image, (int(prev[0]), int(prev[1])), (int(corner[0]), int(corner[1])), color, linewidth)
            prev = corner

    # Draw the sides
    for i in range(4):
        cv2.line(
            image,
            (int(corners.T[i][0]), int(corners.T[i][1])),
            (int(corners.T[i + 4][0]), int(corners.T[i + 4][1])),
            colors[2][::-1],
            linewidth,
        )

    # Draw front (first 4 corners) and rear (last 4 corners) rectangles(3d)/lines(2d)
    draw_rect(image, corners.T[:4], colors[0][::-1], linewidth)
    draw_rect(image, corners.T[4:], colors[1][::-1], linewidth)

def draw_projected_bbox_alpha(img, trans_bbox_img, color=(255, 51, 153)):
    bottom = np.array([[int(trans_bbox_img[0][0]), int(trans_bbox_img[0][1])],
                [int(trans_bbox_img[1][0]), int(trans_bbox_img[1][1])],
                [int(trans_bbox_img[2][0]), int(trans_bbox_img[2][1])],
                [int(trans_bbox_img[3][0]), int(trans_bbox_img[3][1])]])
    top = np.array([[int(trans_bbox_img[4][0]), int(trans_bbox_img[4][1])],
                [int(trans_bbox_img[5][0]), int(trans_bbox_img[5][1])],
                [int(trans_bbox_img[6][0]), int(trans_bbox_img[6][1])],
                [int(trans_bbox_img[7][0]), int(trans_bbox_img[7][1])]])
    left = np.array([[int(trans_bbox_img[4][0]), int(trans_bbox_img[4][1])],
                [int(trans_bbox_img[0][0]), int(trans_bbox_img[0][1])],
                [int(trans_bbox_img[3][0]), int(trans_bbox_img[3][1])],
                [int(trans_bbox_img[7][0]), int(trans_bbox_img[7][1])]])
    right = np.array([[int(trans_bbox_img[5][0]), int(trans_bbox_img[5][1])],
                [int(trans_bbox_img[1][0]), int(trans_bbox_img[1][1])],
                [int(trans_bbox_img[2][0]), int(trans_bbox_img[2][1])],
                [int(trans_bbox_img[6][0]), int(trans_bbox_img[6][1])]])
    front = np.array([[int(trans_bbox_img[0][0]), int(trans_bbox_img[0][1])],
                [int(trans_bbox_img[1][0]), int(trans_bbox_img[1][1])],
                [int(trans_bbox_img[5][0]), int(trans_bbox_img[5][1])],
                [int(trans_bbox_img[4][0]), int(trans_bbox_img[4][1])]])
    rear = np.array([[int(trans_bbox_img[7][0]), int(trans_bbox_img[7][1])],
                [int(trans_bbox_img[6][0]), int(trans_bbox_img[6][1])],
                [int(trans_bbox_img[2][0]), int(trans_bbox_img[2][1])],
                [int(trans_bbox_img[3][0]), int(trans_bbox_img[3][1])]])
    img = cv2.fillPoly(img, [bottom], color)
    img = cv2.fillPoly(img, [top], color)
    img = cv2.fillPoly(img, [left], color)
    img = cv2.fillPoly(img, [right], color)
    img = cv2.fillPoly(img, [front], color)
    img = cv2.fillPoly(img, [rear], color)
    
    return img

def plt_draw_projected_bbox(
    corners,
    colors="green",
):
    """이미지 위에 투영된 3d bbox 그리기"""
    def draw_rect(selected_corners, color):
        prev = selected_corners[-1]
        for corner in selected_corners:
            plt.plot([int(prev[0]), int(corner[0])], [int(prev[1]), int(corner[1])], color)
            prev = corner

    # Draw the sides
    for i in range(4):
        plt.plot(
            [int(corners.T[i][0]), int(corners.T[i + 4][0])],
            [int(corners.T[i][1]), int(corners.T[i + 4][1])],
            colors,
        )

    # Draw front (first 4 corners) and rear (last 4 corners) rectangles(3d)/lines(2d)
    draw_rect(corners.T[:4], colors)
    draw_rect(corners.T[4:], colors)