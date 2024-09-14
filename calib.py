# -*- coding: utf-8 -*-
"""
author:     zhikang.zeng
time  :     2024-07-24 15:02
"""
import argparse
import glob
import os
import json
import cv2
import numpy as np
from utils.str_utils import colormsg

parser = argparse.ArgumentParser(description="Stereo Calib")
parser.add_argument("--raw_dir", type=str, default=r'./data/calib_imgs/raw', help='raw image dir')
parser.add_argument("--row", type=int, default=12, help='the number of squares in a row of the chessboard')
parser.add_argument("--col", type=int, default=9, help='the number of squares in a col chessboard')
parser.add_argument("--block_size", type=int, default=100, help='chessboard block size')
args = parser.parse_args()


class StereoCalib:
    def __init__(self, path_json=None):
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

        if path_json is not None:
            self.load_json(path_json)

    def save_json(self, path_save=None):
        if path_save is None:
            path_save = self.path_save

        if path_save is None:
            raise ValueError('保存路径未指定！')

        self.res_calib = {
            'Stereo': {
                'F': self.Focal,
                'B': self.B,
                'Cx': self.cx,
                'Cy': self.cy,
                'Doffs': self.doffs,
                'Reprojection Error': self.reproj_err_stereo,
                'Depth': 'Depth = F * B / (Disparity + Doffs)'
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

        if path_save is not None:
            with open(path_save, 'w', encoding='utf-8') as f:
                json.dump(self.res_calib, f, indent=4, ensure_ascii=False)
            print(f'Save Json: {os.path.abspath(path_save)}')

    def save_yaml(self, path_save=None):
        if path_save is None:
            path_save = self.path_save

        if path_save is None:
            raise ValueError('保存路径未指定！')

        txt_data = f"""%YAML:1.0
stereo0:
  cam0:
    cam_overlaps: [1]
    camera_model: pinhole
    distortion_coeffs: [{self.distort_l[0, 0]}, {self.distort_l[0, 1]}, {self.distort_l[0, 2]}, {self.distort_l[0, 3]}, {self.distort_l[0, 4]}, {self.distort_l[0, 5]}, {self.distort_l[0, 6]}, {self.distort_l[0, 7]}]
    distortion_model: radtan
    intrinsics: [{self.intr_l[0, 0]}, {self.intr_l[1, 1]}, {self.intr_l[0, 2]}, {self.intr_l[1, 2]}]
    resolution: [1280, 640]
    rostopic: /cam0/image_raw
  cam1:
    T_cn_cnm1:
      - [{self.R[0, 0]}, {self.R[0, 1]}, {self.R[0, 2]}, {self.t[0, 0] / 1000}]
      - [{self.R[1, 0]}, {self.R[1, 1]}, {self.R[1, 2]}, {self.t[1, 0] / 1000}]
      - [{self.R[2, 0]}, {self.R[2, 1]}, {self.R[2, 2]}, {self.t[2, 0] / 1000}]
      - [0.0, 0.0, 0.0, 1.0]
    cam_overlaps: [0]
    camera_model: pinhole
    distortion_coeffs: [{self.distort_r[0, 0]}, {self.distort_r[0, 1]}, {self.distort_r[0, 2]}, {self.distort_r[0, 3]}, {self.distort_r[0, 4]}, {self.distort_r[0, 5]}, {self.distort_r[0, 6]}, {self.distort_r[0, 7]}]
    distortion_model: radtan
    intrinsics: [{self.intr_r[0, 0]}, {self.intr_r[1, 1]}, {self.intr_r[0, 2]}, {self.intr_r[1, 2]}]
    resolution: [1280, 640]
    rostopic: /cam1/image_raw
"""

        if path_save is not None:
            with open(path_save, 'w', encoding='utf-8') as f:
                f.write(txt_data)
            print(f'Save yaml: {os.path.abspath(path_save)}')

    def prt_stereo_param(self):
        print(f'-- F B cx cy doffs: {self.Focal}, {self.B * 1}, {self.cx}, {self.cy}, {self.doffs}')

    def calc_map(self):
        if self.intr_l is None:
            raise ValueError('内外参未计算！')

        # 计算相机的修正变换
        size = (self.width_l, self.height_l)
        flags = cv2.CALIB_ZERO_DISPARITY  # 主点一致
        # flags = 0  # 主点不一致
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(self.intr_l, self.distort_l,
                                                    self.intr_r, self.distort_r,
                                                    size, self.R, self.t, flags=flags)
        self.Focal, self.B, self.cx, self.cy, self.doffs = Q[2, 3], 1 / Q[3, 2], -Q[0, 3], -Q[1, 3], Q[3, 3] / Q[3, 2]

        # 计算映射矩阵LUT，CV_32FC2时map2为空，CV_16SC2时map2为提升定点精度的查找表
        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(self.intr_l, self.distort_l, R1, P1, size, cv2.CV_32FC2)
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(self.intr_r, self.distort_r, R2, P2, size, cv2.CV_32FC2)

        size = (self.width_l + 64, self.height_l + 64)

    def rectify_img(self, img_l, img_r):
        if self.map1_l is None:
            self.calc_map()

        if isinstance(img_l, str) and isinstance(img_r, str):
            # img_l = cv2.imread(img_l, cv2.IMREAD_GRAYSCALE)
            # img_r = cv2.imread(img_r, cv2.IMREAD_GRAYSCALE)
            img_l = cv2.imdecode(np.fromfile(img_l, dtype=np.uint8), -1)
            img_r = cv2.imdecode(np.fromfile(img_r, dtype=np.uint8), -1)

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

        # 单目标定，得到重投影误差、内参、畸变参、外参Rt
        calib_flags = None
        # calib_flags = cv2.CALIB_FIX_K3
        # calib_flags = cv2.CALIB_RATIONAL_MODEL
        reproj_err_l, self.intr_l, self.distort_l, R_l, t_l = cv2.calibrateCamera(pts_world, pts_img_l, size, None,
                                                                                  None, flags=calib_flags)
        reproj_err_r, self.intr_r, self.distort_r, R_r, t_r = cv2.calibrateCamera(pts_world, pts_img_r, size, None,
                                                                                  None, flags=calib_flags)
        print(f'-- 左图重投影误差: {reproj_err_l}, 右图重投影误差: {reproj_err_r}')

        # 立体标定
        # flags = 0
        # flags |= cv2.CALIB_FIX_INTRINSIC  # 不改变内参，只求解R、T、E、F
        # flags |= cv2.CALIB_USE_INTRINSIC_GUESS  # 优化内参
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH  # 保持两相机的焦距一致，若有CALIB_FIX_PRINCIPAL_POINT，则主点也一致
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT  # 不改变cx、cy，可能导致内参不准，一般不启用
        # flags |= cv2.CALIB_RATIONAL_MODEL  # 启用k4、k5、k6
        stereo_calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        # stereo_calib_flags = cv2.CALIB_FIX_INTRINSIC
        # stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
        stereo_calib_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL
        self.reproj_err_stereo, self.intr_l, self.distort_l, self.intr_r, self.distort_r, \
            self.R, self.t, self.E, self.F = cv2.stereoCalibrate(pts_world, pts_img_l, pts_img_r,
                                                                 self.intr_l, self.distort_l, self.intr_r,
                                                                 self.distort_r,
                                                                 size, criteria=stereo_calib_criteria,
                                                                 flags=stereo_calib_flags)

        print('-- left camera matrix:\n', self.intr_l)
        print('-- left distortion coeffs:', self.distort_l)
        print('-- right camera matrix:\n', self.intr_r)
        print('-- right distortion coeffs:', self.distort_r)
        print('-- R:\n', self.R)
        print('-- T:\n', self.t)
        print(f'-- 双目标定重投影误差: {self.reproj_err_stereo}')
        self.calc_map()
        print('-- ======================= Stereo Calib End =======================')


if __name__ == '__main__':
    # 将combine的图像拆分成左右图像
    print('=> =================== 1 ====================')
    # =========== 需要设置的参数 ===========
    raw_dir = args.raw_dir
    row = args.row
    col = args.col
    block_size = args.block_size
    print(colormsg(f'=> param: raw_dir={raw_dir}, row={row}, col={col}, block_size={block_size}'))
    # =========== 需要设置的参数 ===========
    mode = 1
    os.makedirs(rf'{raw_dir}/../chessboard', exist_ok=True)
    for raw_filename in os.listdir(raw_dir):
        print('=> =================================')
        print(f'=> {raw_filename[-8:]}')
        raw_filepath = os.path.join(raw_dir, raw_filename)
        raw_img = cv2.imread(raw_filepath, cv2.IMREAD_GRAYSCALE)
        height, width = raw_img.shape
        print(f'=> height: {height}, width: {width}')
        if mode == 1:
            left_img = raw_img[:height // 2, :]
            right_img = raw_img[height // 2:, :]
        else:
            right_img = raw_img[:height // 2, :]
            left_img = raw_img[height // 2:, :]
        os.makedirs(rf'{raw_dir}/../left', exist_ok=True)
        os.makedirs(rf'{raw_dir}/../right', exist_ok=True)
        cv2.imwrite(rf'{raw_dir}/../left/left{raw_filename[-8:]}', left_img)
        cv2.imwrite(rf'{raw_dir}/../right/right{raw_filename[-8:]}', right_img)
        print('=> =================================')

    # 将标定，注意设置左右图像文件夹和标定板行列以及方块大小，这里是12行9列，每个方块100mm
    print('=> =================== 2 ====================')
    sc = StereoCalib()
    sc.calib(dir_l=rf'{raw_dir}/../left', dir_r=rf'{raw_dir}/../right', row=row, col=col, block_size=block_size)
    sc.prt_stereo_param()
    sc.save_json(rf'{raw_dir}/../calib.json')
    sc.save_yaml(rf'{raw_dir}/../stereo_8.yaml')

    # 极线矫正，注意读入的图像目录
    print('=> =================== 3 ====================')
    # raw_dir = r'./data/calc_disp-0819/raw'
    # for raw_filename in os.listdir(raw_dir):
    #     print('=> =================================')
    #     print(f'=> {raw_filename[-8:]}')
    #     if 'depth' in raw_filename: continue
    #     raw_filepath = os.path.join(raw_dir, raw_filename)
    #     raw_img = cv2.imread(raw_filepath, cv2.IMREAD_GRAYSCALE)
    #     height, width = raw_img.shape
    #     print(f'=> height: {height}, width: {width}')
    #     left_img = raw_img[:height//2, :]
    #     right_img = raw_img[height//2:, :]
    #     os.makedirs(rf'./{raw_dir}/../left', exist_ok=True)
    #     os.makedirs(rf'./{raw_dir}/../right', exist_ok=True)
    #     cv2.imwrite(rf'./{raw_dir}/../left/left{raw_filename[-8:]}', left_img)
    #     cv2.imwrite(rf'./{raw_dir}/../right/right{raw_filename[-8:]}', right_img)
    #     print('=> =================================')

    print('=> =================== 4 ====================')
    left_img_filepaths = []
    right_img_filepaths = []
    for filepath, dirnames, filenames in os.walk(rf'{raw_dir}/..'):
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

        result_dir = os.path.join(filedir, '..', 'rectify')
        os.makedirs(result_dir, exist_ok=True)

        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-left.png'), img_l_rectified)
        cv2.imwrite(os.path.join(result_dir, f'{i + 1:>03}-right.png'), img_r_rectified)
        # break

    print('=================================================================================')
    if sc.reproj_err_stereo < 0.5:
        print(colormsg('=> 重投影误差小于0.5，标定成功，请确认rectify目录的图像是否矫正成功', color='green'))
    else:
        print(colormsg('=> 重投影误差大于0.5，标定失败，请重新采集棋盘格图像'))
    print('=================================================================================')
