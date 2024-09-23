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
time  :     2024-09-14 16:32
"""
import time


# 打印彩色字符
def colormsg(msg: str, color: str = "red", timestamp: bool = False):
    str = ""
    if timestamp:
        str += time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + "  "
    if color == "red":
        str += "\033[1;31;40m"
    elif color == "green":
        str += "\033[1;32;40m"
    elif color == "yellow":
        str += "\033[1;33;40m"
    else:
        return msg
    str += msg + "\033[0m"
    return str
