# -*- coding: utf-8 -*-
"""
author:     zhikang.zeng
time  :     2024-07-24 20:16
"""
import os
import timeit
import cv2
import numpy as np
from utils.pfm_utils import writePFM
from utils.pseudo_color_utils import apply_color_to_ndarray
from utils.point_cloud_utils import save_depth_to_cloud, save_depth_to_color_cloud

if __name__ == '__main__':
    root_dir = r'./data/calc_disp/rectify'

    left_img_filepaths = []
    right_img_filepaths = []
    for filepath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if any(name in filename for name in ('sgm', 'disp', 'depth', 'scene_left.png', 'conf')):
                continue
            tmp_path = (os.path.join(filepath, filename))
            if 'THv' in tmp_path or 'SGM' in tmp_path:
                continue
            if 'left.png' in tmp_path:
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('left.png', 'right.png')
                right_img_filepaths.append(tmp_path)
            if 'spec-l.png' in tmp_path:
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('spec-l.png', 'spec-r.png')
                right_img_filepaths.append(tmp_path)
            # if 'rgb-l.png' in tmp_path:
            #     left_img_filepaths.append(tmp_path)
            #     tmp_path = tmp_path.replace('rgb-l.png', 'rgb-r.png')
            #     right_img_filepaths.append(tmp_path)
            if 'left_ir.png' in tmp_path:
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('left_ir.png', 'right_ir.png')
                right_img_filepaths.append(tmp_path)
            if 'left_pure_ir.png' in tmp_path:
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('left_pure_ir.png', 'right_pure_ir.png')
                right_img_filepaths.append(tmp_path)
            if 'IR_LEFT' in tmp_path and '.png' in tmp_path:
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('IR_LEFT', 'IR_RIGHT')
                right_img_filepaths.append(tmp_path)
            if 'ir_bs_640_400' in tmp_path and tmp_path.endswith('_l.png'):
                left_img_filepaths.append(tmp_path)
                tmp_path = tmp_path.replace('_l.png', '_r.png')
                right_img_filepaths.append(tmp_path)
    assert len(left_img_filepaths) == len(right_img_filepaths) and len(left_img_filepaths) > 0

    left_img_filepaths = left_img_filepaths[::-1]
    right_img_filepaths = right_img_filepaths[::-1]

    for i in range(len(left_img_filepaths)):
        left_dir, left_filename = os.path.split(left_img_filepaths[i])

        to_depth = True
        f, b, cx, cy = 465.2104178592821, 69.82928990313776, 870.4762115478516, 221.603515625
        print(f'=> f: {f}, b: {b}, cx: {cx}, cy: {cy}')

        cv2.namedWindow('dispcolor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('dispcolor', 640, 400)

        start = timeit.default_timer()

        # imgLeft = cv2.imread(left_img_filepaths[i], cv2.IMREAD_GRAYSCALE)
        # imgRight = cv2.imread(right_img_filepaths[i], cv2.IMREAD_GRAYSCALE)
        print(f'{os.path.split(left_img_filepaths[i])[1]} {os.path.split(right_img_filepaths[i])[1]}')
        try:
            imgLeft = cv2.imdecode(np.fromfile(left_img_filepaths[i], dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            imgRight = cv2.imdecode(np.fromfile(right_img_filepaths[i], dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        except Exception as e:
            print(f'!!! {e}')
            continue

        # SGM 计算视差
        numDisparities, blockSize, channel = 192, 7, 1
        left_ext = numDisparities
        right_ext = 0
        left_pad = cv2.copyMakeBorder(imgLeft, top=0, bottom=0, left=left_ext, right=right_ext,
                                      borderType=cv2.BORDER_CONSTANT, value=0)
        right_pad = cv2.copyMakeBorder(imgRight, top=0, bottom=0, left=left_ext, right=right_ext,
                                       borderType=cv2.BORDER_CONSTANT, value=0)
        sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=numDisparities, blockSize=blockSize,
                                     P1=8 * channel * blockSize * blockSize,
                                     P2=32 * channel * blockSize * blockSize,
                                     disp12MaxDiff=3, preFilterCap=None, uniquenessRatio=1,
                                     speckleWindowSize=100, speckleRange=3, mode=cv2.StereoSGBM_MODE_HH)
        disparity = sgbm.compute(left_pad, right_pad)
        disparity = disparity[:, left_ext:]

        # 视差 / 16
        disparity = disparity.astype(np.float32)
        disparity = disparity / 16
        disparity[disparity < 0] = 0

        # 根据阈值去除噪声
        mask1 = np.where((disparity > 20) & (disparity < 500), 1, 0)
        # disparity *= mask1

        thresh, mask2 = cv2.threshold(imgLeft, thresh=30, maxval=1, type=cv2.THRESH_BINARY)
        # disparity *= mask2

        disparity_gray = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        cv2.filterSpeckles(disparity_gray, newVal=0, maxSpeckleSize=1000, maxDiff=3)
        _, mask3 = cv2.threshold(disparity_gray, 0, 1, cv2.THRESH_BINARY)
        disparity *= mask3
        print('最大视差: {}'.format(disparity.max()))
        print('最小视差: {}'.format(np.min(disparity[disparity > 0])))
        print('平均视差: {}'.format(np.mean(disparity[disparity > 0])))

        result_dir = rf'{left_dir}\SGBM_BlockSize{blockSize}'
        if not os.path.exists(result_dir):
            print(f'create result path: {result_dir}')
            os.mkdir(result_dir)
        writePFM(f'{result_dir}/sgm_disparity_{left_filename.split(".")[0]}.pfm', disparity)

        # disparity = cv2.medianBlur(disparity, 5)
        if to_depth:
            depth = f * b / (disparity + 1e-10)
            # depth = f * b / (disparity + doffs + 1e-10)
            depth[depth > 20000] = 0
            depth[disparity == 0] = 0
            depthcolor = apply_color_to_ndarray(depth, text_name='depth', fontScale=0.7)
            if cx != 0 and cy != 0:
                save_depth_to_cloud(f'{result_dir}/sgm_cloud_{left_filename.split(".")[0]}', depth, f, cx, cy)
                save_depth_to_color_cloud(f'{result_dir}/sgm_color_cloud_{left_filename.split(".")[0]}', depth, cv2.cvtColor(imgLeft, cv2.COLOR_GRAY2RGB), f, cx, cy)
            writePFM(f'{result_dir}/sgm_depth_{left_filename.split(".")[0]}.pfm', depth)
            # cv2.imwrite(f'{result_dir}/sgm_depth_{left_filename.split(".")[0]}.png', depthcolor)
            cv2.imencode('.png', depthcolor)[1].tofile(f'{result_dir}/sgm_depth_{left_filename.split(".")[0]}.png')
            print('最大深度: {}'.format(depth.max()))
            print('最小深度: {}'.format(np.min(depth[depth > 0])))
            print('平均深度: {}'.format(np.mean(depth[depth > 0])))

        # 显示&保存结果
        # disparity = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        dispcolor = apply_color_to_ndarray(disparity, text_name='disp', fontScale=0.7)

        # cv2.imwrite(f'{result_dir}/sgm_disparity_{left_filename.split(".")[0]}.png', dispcolor)
        cv2.imencode('.png', dispcolor)[1].tofile(f'{result_dir}/sgm_disparity_{left_filename.split(".")[0]}.png')
        # cv2.imwrite(f'{result_dir}/sgm_disparity_{i * 2}_mask1.png',
        #             cv2.normalize(mask1, None, 0, 255, cv2.NORM_MINMAX))
        # cv2.imwrite(f'{result_dir}/sgm_disparity_{i * 2}_mask2.png',
        #             cv2.normalize(mask2, None, 0, 255, cv2.NORM_MINMAX))
        # cv2.imwrite(f'{result_dir}/sgm_disparity_{i * 2}_mask3.png',
        #             cv2.normalize(mask3, None, 0, 255, cv2.NORM_MINMAX))
        # cv2.imwrite(f'{result_dir}/sgm_disparity_{i * 2}_mask123.png',
        #             cv2.normalize(mask1 * mask2 * mask3, None, 0, 255, cv2.NORM_MINMAX))
        cv2.imshow('dispcolor', dispcolor)
        cv2.waitKey(30)

        end = timeit.default_timer()
        print(f'Running time: {end - start} Seconds')
        print('=========================================================')
        # break
