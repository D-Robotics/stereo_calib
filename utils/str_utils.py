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
