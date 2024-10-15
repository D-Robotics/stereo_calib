# 双目相机标定操作手册

本手册介绍在`RDK X5`上基于`OpenCV`的双目相机标定方法。

## 1、准备工作

- RDK X5 开发板。如无特别说明，以下命令均在RDK X5开发板上进行操作。
- 双目相机模组。
- 棋盘格图像，建议尺寸在100cm*100cm以上。

## 2、安装依赖

### 安装`opencv-python`和`open3d`

```shell
pip install opencv-python
```

### 安装ROS2软件包编译工具

```shell
sudo apt install ros-dev-tools
```

### 下载和编译双目图像数据采集工具

```shell
git clone https://github.com/D-Robotics/hobot_stereonet_utils.git
colcon build --packages-select hobot_stereonet_utils
```

## 3、采集数据

- 在终端1启动双目相机：
  
  ```shell
  source /opt/tros/humble/setup.bash
  source install/local_setup.bash
  ros2 launch hobot_stereonet_utils test_mipi_cam.launch.py
  ```

- 在终端2启动双目图像采集工具：
  
  ```shell
  source /opt/tros/humble/setup.bash
  source install/local_setup.bash
  # 创建数据存放目录
  mkdir -p /userdata/data/calib_imgs/raw
  ros2 run hobot_stereonet_utils save_stereo_img --ros-args -p dir:=/userdata/data/calib_imgs/raw
  ```

采集数据的过程中，PC端浏览器输入`http://ip:8000`地址查看实时的双目图像，用于确认图像是否正确。

在终端2下，按`Enter`键采集一张图像，图像保存在指定的`/userdata/data/calib_imgs/raw`路径下。

**采集注意事项**

- 采集图像时需要保证**相机和棋盘格是静止状态**，否则容易标定失败。
- 采集的距离视棋盘格大小而定，示例使用的棋盘格大小是120cm X 90cm，采集的距离大概是50cm—200cm，棋盘格应占据图像30%—80%的面积，以确保角点能够清晰地被检测到，如图所示：
 ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_001.png)
 ![combine_020.png](data%2Fcalib_imgs%2Fraw%2Fcombine_020.png)
- 采集的角度大概在-30°~30°即可，可以调整棋盘格角度，也可以调整相机角度，如图所示：
 ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_012.png)
 ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_009.png)
- 需要采集**20张以上**的棋盘格图像。

## 4、启动标定

采集完成后，在终端2，停止双目图像采集工具，运行标定程序：

```shell
python ./stereo_calib/calib.py --raw_dir=/userdata/data/calib_imgs/raw --row=12 --col=9 --block_size=100
```

启动时需要指定采集的图片路径，以及棋盘格尺寸参数。其中`raw_dir`为采集的棋盘格图像目录，`row`、`col`、`block_size`按照棋盘格的尺寸进行设置，具体可查看下图：

![chessboard.png](doc%2Fchessboard.png)

标定完成后，终端输出标定成功或失败提示。如果提示成功，查看`/userdata/data/calib_imgs/rectify`文件夹的图像是否矫正成功。矫正成功后，即可使用生成的标定结果文件`/userdata/data/calib_imgs/stereo_8.yaml`。

## 5、验证标定结果

标定结束后，停止终端1和终端2下的程序，使用上一步生成的标定结果文件`/userdata/data/calib_imgs/stereo_8.yaml`启动双目图像采集：

```shell
source /opt/tros/humble/setup.bash
source install/local_setup.bash
ros2 launch hobot_stereonet_utils test_stereo.launch.py stereo_calib_path:=/userdata/data/calib_imgs/stereo_8.yaml visual_alpha:=4
```

PC端浏览器输入`http://ip:8000`地址查看实时的双目图像，可以看到图像已经矫正成功。

## 6、使用标定结果

将标定结果文件拷贝到TROS双目深度估计算法安装路径，覆盖默认的标定文件，完成标定文件的更新。启动双目深度估计算法时，将会使用新的标定结果文件进行深度估计。

启动终端，执行更新命令：

```shell
cp /userdata/data/calib_imgs/stereo_8.yaml /opt/tros/humble/share/stereonet_model/config/stereo.yaml
```
