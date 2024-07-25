# 双目标定（OpenCV python实现）

## 文件说明

- [calib.py](calib.py)：标定、极线矫正代码实现
- [sgbm.py](sgbm.py)：sgmb算法实现
- [data](data)：数据目录


## 标定流程

需要采集20张以上的棋盘格图像，然后运行[calib.py](calib.py)进行标定

## 标定配置

在[calib.py](calib.py)代码中，有两个标志位可以设置，一个是单目标定的`calib_flags`，另一个是双目标定的`stereo_calib_flags`

目前设置`calib_flags=None`，`stereo_calib_flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL`，标定的重投影误差较小

输出日志如下：

```shell
-- ====================== Stereo Calib Start ======================
-- ./data/calib_imgs/left\left_001.png ./data/calib_imgs/right\right_001.png
-- ./data/calib_imgs/left\left_002.png ./data/calib_imgs/right\right_002.png
-- ./data/calib_imgs/left\left_003.png ./data/calib_imgs/right\right_003.png
-- ./data/calib_imgs/left\left_004.png ./data/calib_imgs/right\right_004.png
-- ./data/calib_imgs/left\left_005.png ./data/calib_imgs/right\right_005.png
-- ./data/calib_imgs/left\left_006.png ./data/calib_imgs/right\right_006.png
-- ./data/calib_imgs/left\left_007.png ./data/calib_imgs/right\right_007.png
-- ./data/calib_imgs/left\left_008.png ./data/calib_imgs/right\right_008.png
-- ./data/calib_imgs/left\left_009.png ./data/calib_imgs/right\right_009.png
-- ./data/calib_imgs/left\left_010.png ./data/calib_imgs/right\right_010.png
-- ./data/calib_imgs/left\left_011.png ./data/calib_imgs/right\right_011.png
-- ./data/calib_imgs/left\left_012.png ./data/calib_imgs/right\right_012.png
-- ./data/calib_imgs/left\left_013.png ./data/calib_imgs/right\right_013.png
-- ./data/calib_imgs/left\left_014.png ./data/calib_imgs/right\right_014.png
-- ./data/calib_imgs/left\left_015.png ./data/calib_imgs/right\right_015.png
-- ./data/calib_imgs/left\left_016.png ./data/calib_imgs/right\right_016.png
-- ./data/calib_imgs/left\left_017.png ./data/calib_imgs/right\right_017.png
-- ./data/calib_imgs/left\left_018.png ./data/calib_imgs/right\right_018.png
-- ./data/calib_imgs/left\left_019.png ./data/calib_imgs/right\right_019.png
-- ./data/calib_imgs/left\left_020.png ./data/calib_imgs/right\right_020.png
-- 左图重投影误差: 4.671930829291836, 右图重投影误差: 4.859185223701936
-- left camera matrix:
 [[522.99618673   0.         640.69232983]
 [  0.         465.18420372 296.49439402]
 [  0.           0.           1.        ]]
-- left distortion coeffs: [[ 6.66822613e-01  8.77969376e-02  1.18545533e-04 -2.71921191e-05
   1.38004091e-03  1.03465050e+00  2.40773908e-01  1.19398848e-02
   0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00
   0.00000000e+00  0.00000000e+00]]
-- right camera matrix:
 [[522.8518854    0.         691.05567142]
 [  0.         465.236632   301.86575619]
 [  0.           0.           1.        ]]
-- right distortion coeffs: [[1.19759498e+00 2.85421520e-01 2.77899058e-05 3.37306409e-04
  6.13442581e-03 1.56688095e+00 6.27666431e-01 4.62568869e-02
  0.00000000e+00 0.00000000e+00 0.00000000e+00 0.00000000e+00
  0.00000000e+00 0.00000000e+00]]
-- R:
 [[ 9.99996943e-01  2.83193997e-04 -2.45645372e-03]
 [-2.80755719e-04  9.99999468e-01  9.92888916e-04]
 [ 2.45673360e-03 -9.92196217e-04  9.99996490e-01]]
-- T:
 [[-6.98292351e+01]
 [ 8.34389817e-02]
 [-2.61871346e-02]]
-- 双目标定重投影误差: 0.2238986518250022
-- ======================= Stereo Calib End =======================
-- F B cx cy doffs: 465.2104178592821, 69.82928990313776, 870.4762115478516, 221.603515625, -0.0
Save Json: D:\1_Code\3_Python\stereo_calib\data\calib_imgs\calib.json
```

## 极线矫正

极线矫正代码也在[calib.py](calib.py)中，也有一个标志位可以设置：`flags = cv2.CALIB_ZERO_DISPARITY`时左右图主点位置一致，`flags = 0`时左右图主点位置不一致，保留更大的fov，此时doffs不等于0

深度计算的公式是：Depth = F * B / (Disparity + Doffs)

## SGMB计算深度

输入矫正后的图像和矫正后的内参，可通过[sgbm.py](sgbm.py)计算视差、深度、点云

