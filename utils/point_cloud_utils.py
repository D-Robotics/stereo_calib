# -*- coding: utf-8 -*-
"""
author:     tianhe
time  :     2022-11-18 13:37
"""
import copy
import numpy as np
import open3d as o3d


def depth_to_points(depth: np.array, f, cx, cy):
    points = []
    X_arr = []
    Y_arr = []
    Z_arr = []
    rows, columns = depth.shape
    x, y = np.meshgrid(np.linspace(0, columns - 1, columns), np.linspace(0, rows - 1, rows))
    X = depth * (x - cx) / f
    Y = depth * (y - cy) / f
    search_rows, search_columns = np.where(depth > 0)
    for i in range(search_rows.shape[0]):
        points.append([
            X[search_rows[i], search_columns[i]],
            Y[search_rows[i], search_columns[i]],
            depth[search_rows[i], search_columns[i]]]
        )
        X_arr.append(X[search_rows[i], search_columns[i]])
        Y_arr.append(Y[search_rows[i], search_columns[i]])
        Z_arr.append(depth[search_rows[i], search_columns[i]])
    return points, np.array(X_arr), np.array(Y_arr), np.array(Z_arr)


def disparity_to_points(disparity: np.array, baseline, f, cx, cy):
    points = []
    rows, columns = disparity.shape
    for y in range(rows):
        for x in range(columns):
            if disparity[y, x] != 0:
                Z = baseline * f / disparity[y, x]
                X = Z * (x - cx) / f
                Y = Z * (y - cy) / f
                points.append([X, Y, Z])
    return points


def save_points_to_ply(filepath, points, write_ascii=False):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filepath, point_cloud, write_ascii=write_ascii)


def save_points_to_txt(filepath, points):
    with open(filepath, 'w') as fp:
        for point in points:
            fp.write(f'{str(point)[1:-1]}\n')


def save_open3d_to_txt(filepath, point_cloud):
    points = convert_open3d_to_points(point_cloud)
    save_points_to_txt(filepath, points)


def save_open3d_to_ply(filepath, point_cloud, write_ascii=False):
    o3d.io.write_point_cloud(filepath, point_cloud, write_ascii=write_ascii)


def convert_points_to_open3d(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud


def convert_open3d_to_points(point_cloud):
    points = []
    for point in point_cloud.points:
        points.append(point.tolist())
    return points


def read_txt_to_points(filepath):
    points = []
    with open(filepath, 'r') as fp:
        while True:
            point = fp.readline()
            if not point:
                break
            points.append(list(eval(point)))
    return points


def read_txt_to_open3d(filepath):
    points = read_txt_to_points(filepath)
    return convert_points_to_open3d(points)


def read_ply_to_open3d(filepath):
    filepath = filepath.encode('gbk')
    return o3d.io.read_point_cloud(filepath, format='ply')


def save_depth_to_cloud(filepath: str, depth, f, cx, cy, depth_trunc=20000, write_ascii=False, cloud_type='ply'):
    if cloud_type == 'ply':
        filepath = filepath + '.ply'
    elif cloud_type == 'txt':
        filepath = filepath + '.txt'
    filepath = filepath.encode('gbk')
    h, w = depth.shape
    depth_o3d = o3d.geometry.Image(depth)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, f, f, cx, cy)
    point_cloud = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, intrinsic,
                                                                  depth_scale=1, depth_trunc=depth_trunc)
    save_open3d_to_ply(filepath, point_cloud, write_ascii)


def save_depth_to_color_cloud(filepath: str, depth, color, f, cx, cy, depth_trunc=20000, write_ascii=False,
                              cloud_type='ply'):
    if cloud_type == 'ply':
        filepath = filepath + '.ply'
    elif cloud_type == 'txt':
        filepath = filepath + '.txt'
    filepath = filepath.encode('gbk')
    h, w = depth.shape
    depth_o3d = o3d.geometry.Image(depth)
    color_o3d = o3d.geometry.Image(color)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color=color_o3d, depth=depth_o3d, depth_scale=1.0,
                                                                    depth_trunc=depth_trunc,
                                                                    convert_rgb_to_intensity=False)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, f, f, cx, cy)
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    save_open3d_to_ply(filepath, point_cloud, write_ascii)


def save_disparity_to_cloud(filepath, disparity, f, b, cx, cy, depth_trunc=20000, write_ascii=False, cloud_type='ply'):
    depth = f * b / (disparity + 1e-10)
    save_depth_to_cloud(filepath, depth, f, cx, cy, depth_trunc, write_ascii, cloud_type)


def draw_two_point_cloud(source, target, transformation=None):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    if transformation is not None:
        source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


if __name__ == '__main__':
    disparity = np.load('./disparity.npy')
    points = disparity_to_points(disparity, baseline=150, f=946.483093, cx=1279 - 700.562195, cy=392.026093)
    # save_point_cloud_ply('./point_cloud.ply', points, write_ascii=True)
    # save_point_cloud_txt('./point_cloud.txt', points)
    point_cloud = read_txt_to_open3d('./point_cloud.txt')
    save_open3d_to_txt('./point_cloud2.txt', point_cloud)
    print('hello')
