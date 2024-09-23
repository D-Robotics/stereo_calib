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
time  :     2022-11-18 11:09
"""
import cv2
import numpy as np


def apply_color_to_ndarray(arr: np.array, put_text_flag=True, text_name=None, fontScale=1.5, nonzero_prop=False, abstract=None):
    # 统计非零元素的数量
    non_zero_count = np.count_nonzero(arr)
    # 计算数组元素总数
    total_elements = arr.size
    arr_metric = arr[np.nonzero(arr)]
    if arr_metric.size == 0:
        min_val, max_val, mean_val, median_val = 0, 0, 0, 0
    else:
        min_val, max_val, mean_val, median_val = \
            np.min(arr_metric), np.max(arr_metric), np.mean(arr_metric), np.median(arr_metric)
    if max_val == 0:
        arr = arr.astype(np.uint8)
    else:
        arr = (arr / arr.max() * 255).astype(np.uint8)
    arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    arr_color = cv2.applyColorMap(arr, cv2.COLORMAP_JET)
    arr_color = np.where(arr == (0, 0, 0), (0, 0, 0), arr_color).astype(np.uint8)
    if put_text_flag:
        cv2.putText(arr_color, f'min {text_name}: {min_val:.2f}', (20, 50), cv2.FONT_HERSHEY_COMPLEX, fontScale,
                    (255, 255, 255), 2)
        cv2.putText(arr_color, f'max {text_name}: {max_val:.2f}', (20, 100), cv2.FONT_HERSHEY_COMPLEX, fontScale,
                    (255, 255, 255), 2)
        cv2.putText(arr_color, f'mean {text_name}: {mean_val:.2f}', (20, 150), cv2.FONT_HERSHEY_COMPLEX, fontScale,
                    (255, 255, 255), 2)
        cv2.putText(arr_color, f'median {text_name}: {median_val:.2f}', (20, 200), cv2.FONT_HERSHEY_COMPLEX, fontScale,
                    (255, 255, 255), 2)
        if nonzero_prop:
            cv2.putText(arr_color, f'nozero {text_name}: {non_zero_count / total_elements:.2f}%', (20, 250),
                        cv2.FONT_HERSHEY_COMPLEX, fontScale,
                        (255, 255, 255), 2)
        if abstract is not None:
            cv2.putText(arr_color, abstract, (20, 300), cv2.FONT_HERSHEY_COMPLEX, fontScale, (255, 255, 255), 2)
    return arr_color
