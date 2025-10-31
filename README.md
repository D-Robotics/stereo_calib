# Stereo Calibration

[Chinese](./README_CN.md) | English

## 1. Document Description

- [calib.py](calib.py): Implementation of calibration and epipolar rectification code
- [sgbm.py](sgbm.py): Implementation of the sgmb algorithm
- [data](data): Data Directory
  
## 2. Install the dependency packages first

```shell
# opencv version MUST >= 4.5
pip install opencv-python
pip install open3d
```

## 3. Run the code again

1. More than 20 checkerboard images need to be collected, and the collected data should be placed in `./data/calib_imgs/raw`  directory, or a custom directory can also be used.
  
  - Note that **when capturing images, ensure that the camera and the checkerboard are in a stationary state**. The checkerboard should be mainly located in the central area of the image, and it is also appropriate to capture some images with the checkerboard located at the edge of the image; otherwise, calibration may easily fail. 
  - The distance for data collection depends on the size of the checkerboard. In the example, the size of the checkerboard calibration target is 120cm * 90cm, and the collection distance is approximately 50cm to 200cm. **The checkerboard should occupy 30% to 80% of the image area** to ensure that the corner points can be clearly detected, as shown in the figure. 
  ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_001.png)
  ![combine_020.png](data%2Fcalib_imgs%2Fraw%2Fcombine_020.png)
  - The collection angle can be approximately between -30° and 30°. You can adjust the angle of the checkerboard or the camera angle, as shown in the figure. 
  ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_012.png)
  ![combine_001.png](data%2Fcalib_imgs%2Fraw%2Fcombine_009.png)
  ![calib_raw.png](doc%2Fcalib_raw.png)
 
2. Then run [calib.py](calib.py) for calibration, where `raw_dir` needs to be set to the directory of the checkerboard images, 
`row`, `col`, `block_size` should be set according to the size of the checkerboard. For details, please refer to the figure below. 
`model` can be set to either `fish` or `pinhole`, depending on the distortion level of your lens, if distortion is kind of heavy,
better to use `fish`. But it is recommended to try both fish and pinhole and choose the one that gives the best result.
The program execution command is as follows 
  
```shell
#  use pinhole
python calib.py --raw_dir=./data/calib_imgs/raw --row=12 --col=9 --block_size=100 --model=pinhole
```
![chessboard.png](doc%2Fchessboard.png)


```shell
#  use fish
python calib.py --raw_dir=./fish_data/raw --row=21 --col=12 --block_size=60 --model=fish
```

![chessboard.png](doc%2Fnew_board.png)

3. After running, the following results will be produced 
  
  |  item  |   description    |
  |  ----  | ----  |
  | chessboard  | chessboard corner render result |
  | left  | original left image |
  | right  | original right image |
  | rectify  | rectified image |
  | calib_xxxx.json  | calibration result in json format |
  | stereo_xxxx.yaml  | calibration result in yaml format |
  
![calib_result.png](doc%2Fcalib_result_eng.png)

4. Need to confirm whether the calibration results are correct, check if the reprojection error in the `calib.json` file is less than 0.5, 
and check if the images in the `rectify` folder have been successfully rectified
  

5. Calibration failure cases
  
- Reprojection error is greater than 0.5 
  
![calib_error1.png](doc%2Fcalib_error1.png)

- The reprojection error is less than 0.5, but the calibration parameters are abnormal, `rectify`
 The images in the folder are completely incorrect 
  
![calib_error1.png](doc%2Fcalib_error2.png)

- The reprojection error is less than 0.5, `rectify`
  There are anomalies in the images in the folder. In this case,
   the calibration parameters are not completely unusable, but they will affect the depth accuracy. For example 
  
![calib_error2.png](doc%2Fcalib_error3.png)

![calib_error2.png](doc%2Fcalib_error4.png)

![calib_error2.png](doc%2Fcalib_error5.png)

![calib_error2.png](doc%2Fcalib_error6.png)

![calib_error2.png](doc%2Fcalib_error7.png)

If the above situation occurs, please recollect the image and then perform calibration 


## 4. Calibration Configuration

In the [calib.py](calib.py) code, there are two flags that can be set, 
one is `calib_flags` for mono calibration, and the other is `stereo_calib_flags` for stereo calibration

Currently set `calib_flags=None`, `stereo_calib_flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL` in `pinhole`, 
and set `stereo_calib_flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW` in `fish`
the reprojection error of the calibration is small.

Output log is as follows:
```shell
-- ====================== Stereo Calib Start ======================
...
-- Left image reprojection error: 0.12949030109517565, Right image reprojection error: 0.12460417066817898
-- left camera matrix:
 [[658.1741247    0.         646.05353639]
 [  0.         658.30601712 548.53666377]
 [  0.           0.           1.        ]]
-- left distortion coeffs: [[-0.0199456 ]
 [-0.00640722]
 [ 0.01037764]
 [-0.00745861]]
-- right camera matrix:
 [[659.09415164   0.         641.17852313]
 [  0.         659.12030536 546.31961247]
 [  0.           0.           1.        ]]
-- right distortion coeffs: [[-0.0193645 ]
 [-0.00300522]
 [ 0.00317018]
 [-0.00358357]]
-- R:
 [[ 0.99999446  0.00302244  0.0013916 ]
 [-0.00301419  0.99997809 -0.00589311]
 [-0.00140938  0.00588888  0.99998167]]
-- T:
 [[-7.96251331e+01]
 [ 1.82031895e-01]
 [ 7.47745585e-02]]
-- Stereo reprojection error: 0.12937212606379386
-- F B cx cy doffs fov_scale: 612.5314060500974, 79.6253762710031, 648.7790623361407, 550.2822714476908, -0.0 0.8
-- ======================= Stereo Calib End =======================
-- F B cx cy doffs: 612.5314060500974, 79.6253762710031, 648.7790623361407, 550.2822714476908, -0.0
Save Json: /home/zhy/c-lab/stereo/stereo_calib/fish_data/calib_fish.json
Save yaml: /home/zhy/c-lab/stereo/stereo_calib/fish_data/stereo_fish.yaml
=> =================== 3 ====================
=> =================== 4 ====================
=> Rectify img: ./fish_data/raw/../left/left_002.png ./fish_data/raw/../right/right_002.png
=> Rectify img: ./fish_data/raw/../left/left_003.png ./fish_data/raw/../right/right_003.png
=> Rectify img: ./fish_data/raw/../left/left_004.png ./fish_data/raw/../right/right_004.png
=> Rectify img: ./fish_data/raw/../left/left_005.png ./fish_data/raw/../right/right_005.png
=> Rectify img: ./fish_data/raw/../left/left_006.png ./fish_data/raw/../right/right_006.png
=> Rectify img: ./fish_data/raw/../left/left_007.png ./fish_data/raw/../right/right_007.png
=> Rectify img: ./fish_data/raw/../left/left_008.png ./fish_data/raw/../right/right_008.png
=> Rectify img: ./fish_data/raw/../left/left_009.png ./fish_data/raw/../right/right_009.png
=> Rectify img: ./fish_data/raw/../left/left_010.png ./fish_data/raw/../right/right_010.png
=> Rectify img: ./fish_data/raw/../left/left_011.png ./fish_data/raw/../right/right_011.png
=> Rectify img: ./fish_data/raw/../left/left_012.png ./fish_data/raw/../right/right_012.png
=> Rectify img: ./fish_data/raw/../left/left_013.png ./fish_data/raw/../right/right_013.png
=> Rectify img: ./fish_data/raw/../left/left_014.png ./fish_data/raw/../right/right_014.png
=> Rectify img: ./fish_data/raw/../left/left_015.png ./fish_data/raw/../right/right_015.png
=> Rectify img: ./fish_data/raw/../left/left_016.png ./fish_data/raw/../right/right_016.png
=> Rectify img: ./fish_data/raw/../left/left_017.png ./fish_data/raw/../right/right_017.png
=> Rectify img: ./fish_data/raw/../left/left_018.png ./fish_data/raw/../right/right_018.png
=> Rectify img: ./fish_data/raw/../left/left_019.png ./fish_data/raw/../right/right_019.png
=> Rectify img: ./fish_data/raw/../left/left_020.png ./fish_data/raw/../right/right_020.png
=> Rectify img: ./fish_data/raw/../left/left_021.png ./fish_data/raw/../right/right_021.png
=> Rectify img: ./fish_data/raw/../left/left_022.png ./fish_data/raw/../right/right_022.png
=> Rectify img: ./fish_data/raw/../left/left_023.png ./fish_data/raw/../right/right_023.png
=> Rectify img: ./fish_data/raw/../left/left_024.png ./fish_data/raw/../right/right_024.png
=> Rectify img: ./fish_data/raw/../left/left_025.png ./fish_data/raw/../right/right_025.png
=================================================================================
=> The reprojection error is less than 0.5, and the calibration is successful.
Please confirm whether the image in the `rectify` directory has been corrected successfully.
=================================================================================
```

## 5. Epipolar rectification

The epipolar rectification code is also in [calib.py](calib.py), and there is also a flag that can be set:  When 
`flags = cv2.CALIB_ZERO_DISPARITY`,  the principal point positions of the left and right images are the same, 
When `flags = 0`,  the principal point positions of the left and right images are different, 
retaining a larger field of view (FOV), and at this time doffs is not equal to 0 

The formula for depth calculation is: Depth = F * B / (Disparity + Doffs) 

## 6. SGMB Calculation Depth

By inputting the corrected image and corrected intrinsic parameters, disparity, depth, 
and point cloud can be calculated via [sgbm.py](sgbm.py) 
