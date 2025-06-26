# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -*- coding: utf-8 -*-
"""
author:     zhikang.zeng
time  :     2024-07-24 15:02
"""
import argparse
import ast
import copy
import glob
import os
import json
import re
import shutil

import cv2
import numpy as np
import yaml

from utils.str_utils import colormsg

parser = argparse.ArgumentParser(description="Stereo Calib")
parser.add_argument("--raw_dir", type=str, default=r'./data/calib_imgs/raw', help='raw image dir')
parser.add_argument("--row", type=int, default=12, help='the number of squares in a row of the chessboard')
parser.add_argument("--col", type=int, default=9, help='the number of squares in a col chessboard')
parser.add_argument("--block_size", type=int, default=100, help='chessboard block size')
parser.add_argument("--left_down", action="store_true",
                    help='left up right down left_down=False; left down right up left_down=True')
parser.add_argument("--rect_w", type=int, default=-1, help='rectify image width')
parser.add_argument("--rect_h", type=int, default=-1, help='rectify image height')
parser.add_argument("--model", type=str, default='pinhole', help='opt [pinhole, fish]')
parser.add_argument("--fov_scale", type=float, default=0.8, help='fisheye rectify fov scale')
args = parser.parse_args()


class StereoCalib:
    def __init__(self):
        self.width_l = self.height_l = None
        self.width_r = self.height_r = None
        self.intr_l = self.distort_l = None
        self.intr_r = self.distort_r = None
        self.R = self.t = None
        self.E = self.F = None
        self.reproj_err_stereo = None

        # 校正后的参数，可用于视差转深度
        self.Focal = self.B = self.cx = self.cy = self.doffs = None

        # 做remap的映射矩阵
        self.map1_l = self.map2_l = None
        self.map1_r = self.map2_r = None

        self.res_calib = None
        self.path_save = None

    def save_json(self, path_save=None):
        if path_save is None:
            path_save = self.path_save

        if path_save is None:
            raise ValueError('The save path is not set!')

        if args.model == 'pinhole':
            self.res_calib = {
                'Stereo': {
                    'F': self.Focal,
                    'B': self.B,
                    'Cx': self.cx,
                    'Cy': self.cy,
                    'Doffs': self.doffs,
                    'Reprojection Error': self.reproj_err_stereo,
                    'Depth': 'Depth = F * B / (Disparity + Doffs)',
                    'Origin Size': f'[{self.width_l}, {self.height_l}]',
                    'Calib Size': f'[{args.rect_w}, {args.rect_h}]',
                    'Model': args.model,
                },
                'Left': {
                    'Width': self.width_l,
                    'Height': self.height_l,
                    'Fx': self.intr_l[0, 0],
                    'Fy': self.intr_l[1, 1],
                    'Cx': self.intr_l[0, 2],
                    'Cy': self.intr_l[1, 2],
                    'K1': self.distort_l[0, 0],
                    'K2': self.distort_l[0, 1],
                    'K3': self.distort_l[0, 4],
                    'P1': self.distort_l[0, 2],
                    'P2': self.distort_l[0, 3],
                    'K4': 0.0 if self.distort_l.size == 5 else self.distort_l[0, 5],
                    'K5': 0.0 if self.distort_l.size == 5 else self.distort_l[0, 6],
                    'K6': 0.0 if self.distort_l.size == 5 else self.distort_l[0, 7],
                    'K': [self.intr_l[0, 0], self.intr_l[1, 1], self.intr_l[0, 2], self.intr_l[1, 2]],
                    'D': self.distort_l.flatten().tolist()
                },
                'Right': {
                    'Width': self.width_r,
                    'Height': self.height_r,
                    'Fx': self.intr_r[0, 0],
                    'Fy': self.intr_r[1, 1],
                    'Cx': self.intr_r[0, 2],
                    'Cy': self.intr_r[1, 2],
                    'K1': self.distort_r[0, 0],
                    'K2': self.distort_r[0, 1],
                    'K3': self.distort_r[0, 4],
                    'P1': self.distort_r[0, 2],
                    'P2': self.distort_r[0, 3],
                    'K4': 0.0 if self.distort_r.size == 5 else self.distort_r[0, 5],
                    'K5': 0.0 if self.distort_r.size == 5 else self.distort_r[0, 6],
                    'K6': 0.0 if self.distort_r.size == 5 else self.distort_r[0, 7],
                    'K': [self.intr_r[0, 0], self.intr_r[1, 1], self.intr_r[0, 2], self.intr_r[1, 2]],
                    'D': self.distort_r.flatten().tolist()
                },
                'Rotate': self.R.squeeze().tolist(),
                'Trans': self.t.squeeze().tolist(),
            }
        elif args.model == 'fish':
            self.res_calib = {
                'Stereo': {
                    'F': self.Focal,
                    'B': self.B,
                    'Cx': self.cx,
                    'Cy': self.cy,
                    'Doffs': self.doffs,
                    'Reprojection Error': self.reproj_err_stereo,
                    'Depth': 'Depth = F * B / (Disparity + Doffs)',
                    'Origin Size': f'[{self.width_l}, {self.height_l}]',
                    'Calib Size': f'[{args.rect_w}, {args.rect_h}]',
                    'Model': args.model,
                    'Fov Scale': args.fov_scale
                },
                'Left': {
                    'Width': self.width_l,
                    'Height': self.height_l,
                    'Fx': self.intr_l[0, 0],
                    'Fy': self.intr_l[1, 1],
                    'Cx': self.intr_l[0, 2],
                    'Cy': self.intr_l[1, 2],
                    'θ1': self.distort_l[0, 0],
                    'θ2': self.distort_l[1, 0],
                    'θ3': self.distort_l[2, 0],
                    'θ4': self.distort_l[3, 0],
                    'K': [self.intr_l[0, 0], self.intr_l[1, 1], self.intr_l[0, 2], self.intr_l[1, 2]],
                    'D': self.distort_l.flatten().tolist()
                },
                'Right': {
                    'Width': self.width_r,
                    'Height': self.height_r,
                    'Fx': self.intr_r[0, 0],
                    'Fy': self.intr_r[1, 1],
                    'Cx': self.intr_r[0, 2],
                    'Cy': self.intr_r[1, 2],
                    'θ1': self.distort_r[0, 0],
                    'θ2': self.distort_r[1, 0],
                    'θ3': self.distort_r[2, 0],
                    'θ4': self.distort_r[3, 0],
                    'K': [self.intr_r[0, 0], self.intr_r[1, 1], self.intr_r[0, 2], self.intr_r[1, 2]],
                    'D': self.distort_r.flatten().tolist()
                },
                'Rotate': self.R.squeeze().tolist(),
                'Trans': self.t.squeeze().tolist(),
            }
        else:
            raise NotImplementedError

        if path_save is not None:
            with open(path_save, 'w', encoding='utf-8') as f:
                json.dump(self.res_calib, f, indent=4, ensure_ascii=False)
            print(f'Save Json: {os.path.abspath(path_save)}')

    def save_yaml(self, path_save=None):
        if path_save is None:
            path_save = self.path_save

        if path_save is None:
            raise ValueError('The save path is not set!')

        if args.model == 'pinhole':
            distort_l_values = [f"{self.distort_l[0, i]}" for i in range(self.distort_l.size) if
                                self.distort_l[0, i] != 0]
            distort_l_string = f"[{', '.join(distort_l_values)}]"
            distort_r_values = [f"{self.distort_r[0, i]}" for i in range(self.distort_r.size) if
                                self.distort_r[0, i] != 0]
            distort_r_string = f"[{', '.join(distort_r_values)}]"
            if self.distort_l.size <= 5:
                distortion_model = 'plumb_bob'
            else:
                distortion_model = 'rational_polynomial'
        elif args.model == 'fish':
            distort_l_values = [f"{self.distort_l[i, 0]}" for i in range(self.distort_l.size)]
            distort_l_string = f"[{', '.join(distort_l_values)}]"
            distort_r_values = [f"{self.distort_r[i, 0]}" for i in range(self.distort_r.size)]
            distort_r_string = f"[{', '.join(distort_r_values)}]"
            distortion_model = 'equidistant'
        else:
            raise NotImplementedError
        txt_data = f"""%YAML:1.0
stereo0:
  cam0:
    cam_overlaps: [1]
    camera_model: {args.model}
    distortion_coeffs: {distort_l_string}
    distortion_model: {distortion_model}
    intrinsics: [{self.intr_l[0, 0]}, {self.intr_l[1, 1]}, {self.intr_l[0, 2]}, {self.intr_l[1, 2]}]
    resolution: [{self.width_l}, {self.height_l}]
    rostopic: /cam0/image_raw
  cam1:
    T_cn_cnm1:
      - [{self.R[0, 0]}, {self.R[0, 1]}, {self.R[0, 2]}, {self.t[0, 0] / 1000}]
      - [{self.R[1, 0]}, {self.R[1, 1]}, {self.R[1, 2]}, {self.t[1, 0] / 1000}]
      - [{self.R[2, 0]}, {self.R[2, 1]}, {self.R[2, 2]}, {self.t[2, 0] / 1000}]
      - [0.0, 0.0, 0.0, 1.0]
    cam_overlaps: [0]
    camera_model: {args.model}
    distortion_coeffs: {distort_r_string}
    distortion_model: {distortion_model}
    intrinsics: [{self.intr_r[0, 0]}, {self.intr_r[1, 1]}, {self.intr_r[0, 2]}, {self.intr_r[1, 2]}]
    resolution: [{self.width_r}, {self.height_r}]
    rostopic: /cam1/image_raw
"""
        if args.model == 'fish':
            txt_data += f'    fov_scale: {args.fov_scale}'

        if path_save is not None:
            with open(path_save, 'w', encoding='utf-8') as f:
                f.write(txt_data)
            print(f'Save yaml: {os.path.abspath(path_save)}')

    def prt_stereo_param(self):
        print(f'-- F B cx cy doffs: {self.Focal}, {self.B * 1}, {self.cx}, {self.cy}, {self.doffs}')

    def calc_map(self):
        if self.intr_l is None:
            raise ValueError('Internal and external parameters are not calculated!')

        # 计算相机的修正变换
        size = (self.width_l, self.height_l)
        if args.rect_w != -1 and args.rect_h != -1:
            new_size = (args.rect_w, args.rect_h)
        else:
            new_size = size
        if args.model == 'pinhole':
            flags = cv2.CALIB_ZERO_DISPARITY  # 主点一致
            # flags = 0  # 主点不一致
            # alpha = -1
            alpha = 0
            # alpha = 0.5
            # alpha = 1
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(self.intr_l, self.distort_l,
                                                        self.intr_r, self.distort_r,
                                                        size, self.R, self.t, flags=flags, alpha=alpha,
                                                        newImageSize=new_size)
            self.Focal, self.B, self.cx, self.cy, self.doffs = Q[2, 3], 1 / Q[3, 2], -Q[0, 3], -Q[1, 3], Q[3, 3] / Q[
                3, 2]

            # 计算映射矩阵LUT，CV_32FC2时map2为空，CV_16SC2时map2为提升定点精度的查找表
            self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.intr_l, self.distort_l, R1, P1, new_size,
                                                                   cv2.CV_32FC2)
            self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.intr_r, self.distort_r, R2, P2, new_size,
                                                                   cv2.CV_32FC2)
        elif args.model == 'fish':
            flags = cv2.fisheye.CALIB_ZERO_DISPARITY  # 主点一致
            balance = 0
            fov_scale = args.fov_scale
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(self.intr_l, self.distort_l, self.intr_r, self.distort_r,
                                                          size, self.R, self.t, flags=flags, newImageSize=new_size,
                                                          balance=balance, fov_scale=fov_scale)
            self.Focal, self.B, self.cx, self.cy, self.doffs = Q[2, 3], 1 / Q[3, 2], -Q[0, 3], -Q[1, 3], Q[3, 3] / Q[
                3, 2]
            print(f'-- F B cx cy doffs fov_scale: {self.Focal}, {self.B * 1}, {self.cx}, {self.cy}, {self.doffs} {fov_scale}')

            # 计算映射矩阵LUT，CV_32FC2时map2为空，CV_16SC2时map2为提升定点精度的查找表
            self.map1_l, self.map2_l = cv2.fisheye.initUndistortRectifyMap(self.intr_l, self.distort_l, R1, P1,
                                                                           new_size, cv2.CV_32F)
            self.map1_r, self.map2_r = cv2.fisheye.initUndistortRectifyMap(self.intr_r, self.distort_r, R2, P2,
                                                                           new_size, cv2.CV_32F)
        else:
            raise NotImplementedError


    def rectify_img(self, img_l, img_r):
        if self.map1_l is None:
            self.calc_map()

        if isinstance(img_l, str) and isinstance(img_r, str):
            # img_l = cv2.imread(img_l, cv2.IMREAD_GRAYSCALE)
            # img_r = cv2.imread(img_r, cv2.IMREAD_GRAYSCALE)
            img_l = cv2.imread(img_l, cv2.IMREAD_COLOR)
            img_r = cv2.imread(img_r, cv2.IMREAD_COLOR)
            # img_l = cv2.imdecode(np.fromfile(img_l, dtype=np.uint8), -1)
            # img_r = cv2.imdecode(np.fromfile(img_r, dtype=np.uint8), -1)

        img_l_rectified = cv2.remap(img_l, self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        img_r_rectified = cv2.remap(img_r, self.map1_r, self.map2_r, cv2.INTER_LINEAR)

        return img_l_rectified, img_r_rectified

    def calib(self, dir_l, dir_r, row=12, col=9, block_size=20):
        """计算内外参及畸变"""
        print('-- ====================== Stereo Calib Start ======================')
        if self.path_save is None:
            self.path_save = dir_l.replace('left', 'stereo-calib-res.json')

        images_l = sorted(glob.glob(f'{dir_l}/*'))
        images_r = sorted(glob.glob(f'{dir_r}/*'))

        points_per_row, points_per_col = row - 1, col - 1

        # 此处假设棋盘格所在的平面为世界坐标系中Z=0的平面
        objpoints = np.zeros((points_per_row * points_per_col, 3), np.float32)  # 棋盘格角点的世界坐标，单位米
        objpoints[:, :2] = np.mgrid[0:points_per_row, 0:points_per_col].T.reshape(-1, 2) * block_size

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # 终止条件

        pts_world = []  # 角点在世界坐标系下的坐标
        pts_img_l = []  # 左图角点在图像坐标系下的坐标
        pts_img_r = []  # 右图角点在图像坐标系下的坐标

        for i in range(len(images_l)):
            print(f'-- {images_l[i]} {images_r[i]}')
            img_l = cv2.imread(images_l[i], cv2.IMREAD_GRAYSCALE)  # show(img_l)
            img_r = cv2.imread(images_r[i], cv2.IMREAD_GRAYSCALE)  # show(img_r)

            # 提取棋盘格角点
            ret_l, corners_l = cv2.findChessboardCorners(img_l, (points_per_row, points_per_col), None)
            ret_r, corners_r = cv2.findChessboardCorners(img_r, (points_per_row, points_per_col), None)

            # 亚像素精确化，对粗提取的角点进行精确化，并添加每幅图的对应3D-2D坐标
            if ret_l and ret_r:
                pts_world.append(objpoints)
                cv2.cornerSubPix(img_l, corners_l, (11, 11), (-1, -1), criteria)
                cv2.cornerSubPix(img_r, corners_r, (11, 11), (-1, -1), criteria)
                pts_img_l.append(corners_l)
                pts_img_r.append(corners_r)

                # 可视化角点
                img_l_c = cv2.cvtColor(img_l, cv2.COLOR_GRAY2BGR)
                img_r_c = cv2.cvtColor(img_r, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(img_l_c, (points_per_row, points_per_col), corners_l, ret_l)
                cv2.drawChessboardCorners(img_r_c, (points_per_row, points_per_col), corners_r, ret_r)
                img_corners_show = np.concatenate((img_l_c, img_r_c), axis=1)
                cv2.imwrite(f'{dir_l}/../chessboard/chessboard_{i + 1}.png', img_corners_show)
                cv2.namedWindow('show corners', 0)
                cv2.resizeWindow('show corners', 640 * 2, 400)
                cv2.imshow('show corners', img_corners_show)
                cv2.waitKey(100)  # 显示100ms
        cv2.destroyAllWindows()

        self.height_l, self.width_l = img_l.shape
        self.height_r, self.width_r = img_r.shape
        size = (self.width_l, self.height_l)

        if args.model == 'pinhole':
            # 单目标定，得到重投影误差、内参、畸变参、外参Rt
            calib_criteria = None
            # calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            # calib_flags = None
            # calib_flags = cv2.CALIB_FIX_K1
            # calib_flags = cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
            calib_flags = cv2.CALIB_FIX_K3
            # calib_flags = cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_ASPECT_RATIO
            # calib_flags = cv2.CALIB_ZERO_TANGENT_DIST
            # calib_flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            # calib_flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
            # calib_flags = cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            # calib_flags = cv2.CALIB_RATIONAL_MODEL
            reproj_err_l, self.intr_l, self.distort_l, R_l, t_l = cv2.calibrateCamera(pts_world, pts_img_l, size, None,
                                                                                      None, flags=calib_flags,
                                                                                      criteria=calib_criteria)
            # reproj_err_r, self.intr_r, self.distort_r, R_r, t_r = cv2.calibrateCamera(pts_world, pts_img_r, size, None,
            #                                                                           None, flags=calib_flags,
            #                                                                           criteria=calib_criteria)
            # calib_flags = None
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K1
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
            # calib_flags = cv2.CALIB_FIX_K3
            calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_ASPECT_RATIO
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL
            # calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            reproj_err_r, self.intr_r, self.distort_r, R_r, t_r = cv2.calibrateCamera(pts_world, pts_img_r, size,
                                                                                      copy.deepcopy(self.intr_l),
                                                                                      copy.deepcopy(self.distort_l),
                                                                                      flags=calib_flags,
                                                                                      criteria=calib_criteria)
            print(f'-- Left image reprojection error: {reproj_err_l}, Right image reprojection error: {reproj_err_r}')

            # 立体标定
            # flags = 0
            # flags |= cv2.CALIB_FIX_INTRINSIC  # 不改变内参，只求解R、T、E、F
            # flags |= cv2.CALIB_USE_INTRINSIC_GUESS  # 优化内参
            # flags |= cv2.CALIB_SAME_FOCAL_LENGTH  # 保持两相机的焦距一致，若有CALIB_FIX_PRINCIPAL_POINT，则主点也一致
            # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT  # 不改变cx、cy，可能导致内参不准，一般不启用
            # flags |= cv2.CALIB_RATIONAL_MODEL  # 启用k4、k5、k6
            # stereo_calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            stereo_calib_criteria = None
            # stereo_calib_flags = None
            # stereo_calib_flags = cv2.CALIB_FIX_PRINCIPAL_POINT
            # stereo_calib_flags = cv2.CALIB_FIX_INTRINSIC
            # stereo_calib_flags = cv2.CALIB_SAME_FOCAL_LENGTH
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS
            # stereo_calib_flags = cv2.CALIB_RATIONAL_MODEL
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K1
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_ASPECT_RATIO
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
            stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL
            # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
            self.reproj_err_stereo, self.intr_l, self.distort_l, self.intr_r, self.distort_r, \
                self.R, self.t, self.E, self.F = cv2.stereoCalibrate(pts_world, pts_img_l, pts_img_r,
                                                                     self.intr_l, self.distort_l, self.intr_r,
                                                                     self.distort_r,
                                                                     size, criteria=stereo_calib_criteria,
                                                                     flags=stereo_calib_flags)
        elif args.model == 'fish':
            pts_world = [pts[None, ...] for pts in pts_world]
            calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            calib_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
            reproj_err_l, self.intr_l, self.distort_l, R_l, t_l = cv2.fisheye.calibrate(pts_world, pts_img_l, size,
                                                                                        None, None, flags=calib_flags,
                                                                                        criteria=calib_criteria)
            calib_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
            reproj_err_r, self.intr_r, self.distort_r, R_r, t_r = cv2.fisheye.calibrate(pts_world, pts_img_r, size,
                                                                                        copy.deepcopy(self.intr_l),
                                                                                        copy.deepcopy(self.distort_l),
                                                                                        flags=calib_flags,
                                                                                        criteria=calib_criteria)
            print(f'-- Left image reprojection error: {reproj_err_l}, Right image reprojection error: {reproj_err_r}')
            stereo_calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            stereo_calib_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
            pts_img_l = [np.squeeze(pts)[None, ...] for pts in pts_img_l]
            pts_img_r = [np.squeeze(pts)[None, ...] for pts in pts_img_r]
            self.reproj_err_stereo, self.intr_l, self.distort_l, self.intr_r, self.distort_r, \
                self.R, self.t, self.E, self.F = cv2.fisheye.stereoCalibrate(pts_world, pts_img_l, pts_img_r,
                                                                             self.intr_l, self.distort_l,
                                                                             self.intr_r, self.distort_r,
                                                                             size, criteria=stereo_calib_criteria,
                                                                             flags=stereo_calib_flags)
        else:
            raise NotImplementedError
        print('-- left camera matrix:\n', self.intr_l)
        print('-- left distortion coeffs:', self.distort_l)
        print('-- right camera matrix:\n', self.intr_r)
        print('-- right distortion coeffs:', self.distort_r)
        print('-- R:\n', self.R)
        print('-- T:\n', self.t)
        print(f'-- Stereo reprojection error: {self.reproj_err_stereo}')
        self.calc_map()
        print('-- ======================= Stereo Calib End =======================')

    def load_json_lh(self, path_json):
        with open(path_json, 'r', encoding='utf-8') as f:
            calib = json.load(f)

        self.width_l, self.height_l = int(calib['width']), int(calib['height'])
        self.width_r, self.height_r = int(calib['width']), int(calib['height'])

        self.intr_l = np.array([
            [float(calib['camera_matrix_left']['data'][0]), float(calib['camera_matrix_left']['data'][1]),
             float(calib['camera_matrix_left']['data'][2])],
            [float(calib['camera_matrix_left']['data'][3]), float(calib['camera_matrix_left']['data'][4]),
             float(calib['camera_matrix_left']['data'][5])],
            [float(calib['camera_matrix_left']['data'][6]), float(calib['camera_matrix_left']['data'][7]),
             float(calib['camera_matrix_left']['data'][8])]
        ])
        self.distort_l = np.array([[
            float(calib['distortion_l']['data'][0]),
            float(calib['distortion_l']['data'][1]),
            float(calib['distortion_l']['data'][6]),
            float(calib['distortion_l']['data'][7]),
            float(calib['distortion_l']['data'][2]),
            float(calib['distortion_l']['data'][3]),
            float(calib['distortion_l']['data'][4]),
            float(calib['distortion_l']['data'][5]),
        ]])

        self.intr_r = np.array([
            [float(calib['camera_matrix_right']['data'][0]), float(calib['camera_matrix_right']['data'][1]),
             float(calib['camera_matrix_right']['data'][2])],
            [float(calib['camera_matrix_right']['data'][3]), float(calib['camera_matrix_right']['data'][4]),
             float(calib['camera_matrix_right']['data'][5])],
            [float(calib['camera_matrix_right']['data'][6]), float(calib['camera_matrix_right']['data'][7]),
             float(calib['camera_matrix_right']['data'][8])]
        ])
        self.distort_r = np.array([[
            float(calib['distortion_r']['data'][0]),
            float(calib['distortion_r']['data'][1]),
            float(calib['distortion_r']['data'][6]),
            float(calib['distortion_r']['data'][7]),
            float(calib['distortion_r']['data'][2]),
            float(calib['distortion_r']['data'][3]),
            float(calib['distortion_r']['data'][4]),
            float(calib['distortion_r']['data'][5]),
        ]])
        # 旋转矩阵和平移向量
        self.R = np.array([
            [float(calib['R']['data'][0]), float(calib['R']['data'][1]), float(calib['R']['data'][2])],
            [float(calib['R']['data'][3]), float(calib['R']['data'][4]), float(calib['R']['data'][5])],
            [float(calib['R']['data'][6]), float(calib['R']['data'][7]), float(calib['R']['data'][8])]
        ])
        self.t = np.array([[float(calib['T']['data'][0]) * 1000], [float(calib['T']['data'][1]) * 1000],
                           [float(calib['T']['data'][2]) * 1000]])
        self.calc_map()
        print(f'-- Load [{path_json}] Success!')

    def load_json(self, path_json):
        with open(path_json, 'r', encoding='utf-8') as f:
            calib = json.load(f)

        self.width_l, self.height_l = calib['Left']['Width'], calib['Left']['Height']
        self.width_r, self.height_r = calib['Right']['Width'], calib['Right']['Height']

        self.intr_l = np.array([
            [calib['Left']['Fx'], 0, calib['Left']['Cx']],
            [0, calib['Left']['Fy'], calib['Left']['Cy']],
            [0, 0, 1]
        ])
        self.distort_l = np.array([[calib['Left']['K1'], calib['Left']['K2'],
                                    calib['Left']['P1'], calib['Left']['P2'], calib['Left']['K3'],
                                    calib['Left']['K4'], calib['Left']['K5'], calib['Left']['K6']
                                    ]])

        self.intr_r = np.array([
            [calib['Right']['Fx'], 0, calib['Right']['Cx']],
            [0, calib['Right']['Fy'], calib['Right']['Cy']],
            [0, 0, 1]
        ])
        self.distort_r = np.array([[calib['Right']['K1'], calib['Right']['K2'],
                                    calib['Right']['P1'], calib['Right']['P2'], calib['Right']['K3'],
                                    calib['Right']['K4'], calib['Right']['K5'], calib['Right']['K6']
                                    ]])
        # 旋转矩阵和平移向量
        self.R = np.array(calib['Rotate'])
        self.t = np.array(calib['Trans'])
        self.calc_map()
        print(f'-- Load [{path_json}] Success!')

    def extract_list_from_line(self, line):
        list_str = re.search(r'\[.*\]', line).group(0)
        return ast.literal_eval(list_str)

    def get_intrinsic_matrix(self, params):
        fx, fy, cx, cy = params
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

    def load_yaml(self, path_yaml):
        # 读取YAML文件
        intrinsics_cam0 = None
        distortion_cam0 = None
        intrinsics_cam1 = None
        distortion_cam1 = None
        T_cn_cnm1 = []
        reading_T = False
        with open(path_yaml, 'r') as f:
            lines = f.readlines()

            for line in lines:
                line = line.strip()
                if line.startswith('intrinsics:'):
                    values = self.extract_list_from_line(line)
                    if intrinsics_cam0 is None:
                        intrinsics_cam0 = values
                    else:
                        intrinsics_cam1 = values
                elif line.startswith('distortion_coeffs:'):
                    values = self.extract_list_from_line(line)
                    if distortion_cam0 is None:
                        distortion_cam0 = values
                    else:
                        distortion_cam1 = values
                elif line.startswith('resolution:'):
                    values = self.extract_list_from_line(line)
                    self.width_l, self.height_l = values[0], values[1]
                    self.width_r, self.height_r = values[0], values[1]
                elif line.startswith('T_cn_cnm1:'):
                    reading_T = True  # 启动读取矩阵的标志
                    T_cn_cnm1 = []  # 清空之前的内容
                elif reading_T:
                    if line.startswith('- ['):
                        row = self.extract_list_from_line(line)
                        T_cn_cnm1.append(row)
                        if len(T_cn_cnm1) == 4:
                            reading_T = False  # 读取完毕
        self.intr_l = self.get_intrinsic_matrix(intrinsics_cam0)
        self.intr_r = self.get_intrinsic_matrix(intrinsics_cam1)
        self.distort_l = np.array(distortion_cam0)
        self.distort_r = np.array(distortion_cam1)
        T = np.array(T_cn_cnm1)
        # 旋转矩阵和平移向量
        self.R = np.array(T[:3, :3])
        self.t = np.array(T[:3, -1] * 1000)
        self.calc_map()
        print(f'-- Load [{path_yaml}] Success!')

    def load_yaml_aceii(self, file_path):
        with open(file_path, "r", encoding="utf-8") as f:
            # 直接使用 YAML 加载
            data = yaml.safe_load(f)

        # 把 YAML 加载出来的列表/嵌套列表转换为 numpy 数组
        K_left = np.array(data["左立体内参矩阵"])
        D_left = np.array(data["左立体畸变系数"][0])  # 注意：它是一个包含一个列表的列表
        K_right = np.array(data["右立体内参矩阵"])
        D_right = np.array(data["右立体畸变系数"][0])
        R = np.array(data["外参旋转矩阵"])
        T = np.array([row[0] for row in data["外参平移矩阵"]])

        self.width_l, self.height_l = 2592, 1944
        self.width_r, self.height_r = 2592, 1944
        self.intr_l = K_left
        self.intr_r = K_right
        self.distort_l = D_left
        self.distort_r = D_right
        self.R = R
        self.t = T
        self.calc_map()
        print(f'-- Load [{file_path}] Success!')

    def load_yaml_cv(self, file_path):
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)

        if not fs.isOpened():
            raise IOError(f"Cannot open file: {file_path}")

        # 读取数值（标量）
        # stereo_rms = fs.getNode("Stereo RMS").real()
        # epl_rms = fs.getNode("Epl RMS").real()
        # intrinsic1_rms = fs.getNode("Intrinsic1 RMS").real()
        # intrinsic2_rms = fs.getNode("Intrinsic2 RMS").real()

        # 读取矩阵
        M1 = fs.getNode("M1").mat()
        D1 = fs.getNode("D1").mat()
        M2 = fs.getNode("M2").mat()
        D2 = fs.getNode("D2").mat()
        R = fs.getNode("R").mat()
        T = fs.getNode("T").mat()
        # R1 = fs.getNode("R1").mat()
        # R2 = fs.getNode("R2").mat()
        # P1 = fs.getNode("P1").mat()
        # P2 = fs.getNode("P2").mat()

        fs.release()

        self.width_l, self.height_l = 1280, 1088
        self.width_r, self.height_r = 1280, 1088
        self.intr_l = M1
        self.intr_r = M2
        self.distort_l = D1
        self.distort_r = D2
        self.R = R
        self.t = T
        self.calc_map()
        print(f'-- Load [{file_path}] Success!')


if __name__ == '__main1__':
    print('=> =================== 1 ====================')
    # calib_file_path = r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\0925-zed-mini\calib.json'
    # calib_file_path = r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\calib_lh_20240926\json\m.json'
    # calib_file_path = r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\calib_lh_20240926\Rectification\calib.json'
    # calib_file_path = r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\1018-lh-1\stereo_4_1_lh_1018.json'  # rectified fx: 468.224548, fy: 468.224548, cx: 648.605103, cy: 298.273071, base_line: :0.069716
    # calib_file_path = r'D:\VirtualBoxShare\0_ros2bag\data0\stereo_livox_imu_bag_20250529_1_ros1-camchain-imucam.yaml'
    # calib_file_path = r'C:\Users\zhikang.zeng\Pictures\Saved Pictures\test\UCO00085_1T02D256B00010.yml'
    calib_file_path = r'D:\VirtualBoxShare\4_stereo_imu_lidar_calib_20250624\2_stereo_calib\stereo_fish_gz_20250624.yaml'
    print(f'=>=== Load calib json: {calib_file_path}')
    sc = StereoCalib()
    sc.load_yaml(calib_file_path)
    # sc.load_json(calib_file_path)
    print('-- left camera matrix:\n', sc.intr_l)
    print('-- left distortion coeffs:', sc.distort_l)
    print('-- right camera matrix:\n', sc.intr_r)
    print('-- right distortion coeffs:', sc.distort_r)
    print('-- R:\n', sc.R)
    print('-- T:\n', sc.t)
    sc.prt_stereo_param()
    # sc.save_yaml(r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\calib_lh_20240926\Rectification\stereo_8_lh.yaml')
    # sc.save_json(r'D:\3_HoBot\3_RDK_X3_X5\14_Stereo\depth-analyse\230ai-stereo-imgs\stereo_8_custom.json')

    # print('=> =================== 2 ====================')
    raw_dir = r'D:\VirtualBoxShare\4_stereo_imu_lidar_calib_20250624\3_lidar_calib\stereo_livox_lidar_imu_bag_20250624_split\images\raw'
    for raw_filename in os.listdir(raw_dir):
        print('=> =================================')
        if 'depth' in raw_filename: continue
        if not raw_filename.endswith(('.png', '.jpg')): continue
        print(f'=> {raw_filename}')
        raw_filepath = os.path.join(raw_dir, raw_filename)
        raw_img = cv2.imread(raw_filepath, cv2.IMREAD_COLOR)
        height, width, _ = raw_img.shape
        print(f'=> height: {height}, width: {width}')
        # left_img = raw_img[height // 2:, :width // 2]
        # right_img = raw_img[height // 2:, width // 2:]
        left_img = raw_img[:height // 2, :]
        right_img = raw_img[height // 2:, :]
        os.makedirs(rf'{raw_dir}/../left', exist_ok=True)
        os.makedirs(rf'{raw_dir}/../right', exist_ok=True)
        cv2.imwrite(rf'{raw_dir}/../left/left{raw_filename[-8:]}', left_img)
        cv2.imwrite(rf'{raw_dir}/../right/right{raw_filename[-8:]}', right_img)
        print('=> =================================')

    # 极线矫正，注意读入的图像目录
    print('=> =================== 3 ====================')
    left_img_filepaths = []
    right_img_filepaths = []
    for filepath, dirnames, filenames in os.walk(rf'{raw_dir}/..'):
        for filename in filenames:
            tmp_path = (os.path.join(filepath, filename))
            if 'left' in tmp_path and 'rectify' not in tmp_path and tmp_path.endswith('.png'):
                left_img_filepaths.append(tmp_path)
            if 'right' in tmp_path and 'rectify' not in tmp_path and tmp_path.endswith('.png'):
                right_img_filepaths.append(tmp_path)

    assert len(left_img_filepaths) == len(right_img_filepaths)
    for i in range(len(left_img_filepaths)):
        filedir, left_filename = os.path.split(left_img_filepaths[i])
        _, right_filename = os.path.split(right_img_filepaths[i])

        print(f'=> Rectify img: {left_img_filepaths[i]} {right_img_filepaths[i]}')
        img_l_rectified, img_r_rectified = sc.rectify_img(left_img_filepaths[i], right_img_filepaths[i])

        result_dir = os.path.join(filedir, '..', 'rectify_kalibr')
        os.makedirs(result_dir, exist_ok=True)

        # cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-left.png'), img_l_rectified)
        # cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-right.png'), img_r_rectified)
        cv2.imwrite(os.path.join(result_dir, f'left{i + 1:>06}.png'), img_l_rectified)
        cv2.imwrite(os.path.join(result_dir, f'right{i + 1:>06}.png'), img_r_rectified)

if __name__ == '__main__':
    # python calib.py --raw_dir=D:\3_HoBot\3_RDK_X3_X5\14_Stereo\0925-zed\raw --row=12 --col=9 --block_size=20
    # python calib.py --raw_dir=D:\3_HoBot\3_RDK_X3_X5\14_Stereo\1009-6\raw
    # python calib.py --raw_dir=D:\3_HoBot\3_RDK_X3_X5\14_Stereo\data\calib_20\raw --row=21 --col=12 --block_size=60
    # python calib.py --raw_dir=D:\3_HoBot\3_RDK_X3_X5\14_Stereo\data\sc1336_raw_720p\raw --row=10 --col=6 --block_size=40 --model=fish
    # 将combine的图像拆分成左右图像
    print('=> =================== 1 ====================')
    assert args.model in ['pinhole', 'fish']
    # =========== 需要设置的参数 ===========
    print(colormsg(f'=> param:'
                   f'=> raw_dir={args.raw_dir}\n'
                   f'=> row={args.row}\n'
                   f'=> col={args.col}\n'
                   f'=> block_size={args.block_size}\n'
                   f'=> left_down={args.left_down}\n'
                   f'=> rect_w={args.rect_w}\n'
                   f'=> rect_h={args.rect_h}\n'
                   f'=> model={args.model}\n'
                   f'=> fov_scale={args.fov_scale}\n'
                   ))
    # =========== 需要设置的参数 ===========
    if os.path.isdir(rf'{args.raw_dir}/../chessboard'):
        shutil.rmtree(rf'{args.raw_dir}/../chessboard')
    if os.path.isdir(rf'{args.raw_dir}/../left'):
        shutil.rmtree(rf'{args.raw_dir}/../left')
    if os.path.isdir(rf'{args.raw_dir}/../right'):
        shutil.rmtree(rf'{args.raw_dir}/../right')
    if os.path.isdir(rf'{args.raw_dir}/../rectify_{args.model}'):
        shutil.rmtree(rf'{args.raw_dir}/../rectify_{args.model}')
    os.makedirs(rf'{args.raw_dir}/../chessboard', exist_ok=True)
    for raw_filename in os.listdir(args.raw_dir):
        print('=> -------------------------------')
        print(f'=> {raw_filename}')
        raw_filepath = os.path.join(args.raw_dir, raw_filename)
        raw_img = cv2.imread(raw_filepath, cv2.IMREAD_GRAYSCALE)
        height, width = raw_img.shape
        print(f'=> height: {height}, width: {width}')
        if args.left_down:
            right_img = raw_img[:height // 2, :]
            left_img = raw_img[height // 2:, :]
        else:
            left_img = raw_img[:height // 2, :]
            right_img = raw_img[height // 2:, :]
        os.makedirs(rf'{args.raw_dir}/../left', exist_ok=True)
        os.makedirs(rf'{args.raw_dir}/../right', exist_ok=True)
        cv2.imwrite(rf'{args.raw_dir}/../left/left{raw_filename[-8:]}', left_img)
        cv2.imwrite(rf'{args.raw_dir}/../right/right{raw_filename[-8:]}', right_img)

    # 将标定，注意设置左右图像文件夹和标定板行列以及方块大小，这里是12行9列，每个方块100mm
    print('=> =================== 2 ====================')
    sc = StereoCalib()
    sc.calib(dir_l=rf'{args.raw_dir}/../left', dir_r=rf'{args.raw_dir}/../right', row=args.row, col=args.col,
             block_size=args.block_size)
    sc.prt_stereo_param()
    sc.save_json(rf'{args.raw_dir}/../calib_{args.model}.json')
    sc.save_yaml(rf'{args.raw_dir}/../stereo_{args.model}.yaml')

    # 极线矫正，注意读入的图像目录
    print('=> =================== 3 ====================')
    # raw_dir = r'./data/calc_disp-0819/raw'
    # for raw_filename in os.listdir(raw_dir):
    #     print('=> =================================')
    #     print(f'=> {raw_filename[-8:]}')
    #     if 'depth' in raw_filename: continue
    #     raw_filepath = os.path.join(raw_dir, raw_filename)
    #     raw_img = cv2.imread(raw_filepath, cv2.IMREAD_COLOR)
    #     height, width = raw_img.shape
    #     print(f'=> height: {height}, width: {width}')
    #     left_img = raw_img[:height//2, :]
    #     right_img = raw_img[height//2:, :]
    #     os.makedirs(rf'{raw_dir}/../left', exist_ok=True)
    #     os.makedirs(rf'{raw_dir}/../right', exist_ok=True)
    #     cv2.imwrite(rf'{raw_dir}/../left/left{raw_filename[-8:]}', left_img)
    #     cv2.imwrite(rf'{raw_dir}/../right/right{raw_filename[-8:]}', right_img)
    #     print('=> =================================')

    print('=> =================== 4 ====================')
    left_img_filepaths = []
    right_img_filepaths = []
    for filepath, dirnames, filenames in os.walk(rf'{args.raw_dir}/..'):
        # for filepath, dirnames, filenames in os.walk(rf'D:\3_HoBot\3_RDK_X3_X5\19_GZ\19700101_003613'):
        for filename in filenames:
            tmp_path = (os.path.join(filepath, filename))
            if 'left' in tmp_path and 'rectify' not in tmp_path:
                left_img_filepaths.append(tmp_path)
            if 'right' in tmp_path and 'rectify' not in tmp_path:
                right_img_filepaths.append(tmp_path)

    assert len(left_img_filepaths) == len(right_img_filepaths)
    for i in range(len(left_img_filepaths)):
        filedir, left_filename = os.path.split(left_img_filepaths[i])
        _, right_filename = os.path.split(right_img_filepaths[i])

        print(f'=> Rectify img: {left_img_filepaths[i]} {right_img_filepaths[i]}')
        img_l_rectified, img_r_rectified = sc.rectify_img(left_img_filepaths[i], right_img_filepaths[i])

        result_dir = os.path.join(filedir, '..', f'rectify_{args.model}')
        os.makedirs(result_dir, exist_ok=True)

        # cv2.imwrite(os.path.join(result_dir, f'left{i + 1:>06}.png'), img_l_rectified)
        # cv2.imwrite(os.path.join(result_dir, f'right{i + 1:>06}.png'), img_r_rectified)

        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-left.png'), img_l_rectified)
        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-right.png'), img_r_rectified)
        img_l_rectified = np.where(img_l_rectified == (0, 0, 0), (0, 0, 255), img_l_rectified)
        img_r_rectified = np.where(img_r_rectified == (0, 0, 0), (0, 0, 255), img_r_rectified)
        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-visual-left.png'), img_l_rectified)
        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-visual-right.png'), img_r_rectified)
        # break

    print('=================================================================================')
    if sc.reproj_err_stereo < 0.5:
        print(colormsg(
            '=> The reprojection error is less than 0.5, and the calibration is successful. Please confirm whether the image in the `rectify` directory has been corrected successfully.',
            color='green'))
    else:
        print(colormsg(
            '=> The reprojection error is greater than 0.5, and the calibration failed. Please collect the checkerboard image again..'))
    print('=================================================================================')
