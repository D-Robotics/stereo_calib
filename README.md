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
`row`, `col`, `block_size` should be set according to the size of the checkerboard. For details, please refer to the figure below. The program execution command is as follows 
  
```shell
python calib.py --raw_dir=./data/calib_imgs/raw --row=12 --col=9 --block_size=100
```

![chessboard.png](doc%2Fchessboard.png)

3. After running, the following results will be produced 
  
![calib_result.png](doc%2Fcalib_result.png)

4. Need to confirm whether the calibration results are correct, check if the reprojection error in the `calib.json` file is less than 0.5, 
and check if the images in the `rectify` folder have been successfully rectified
  
![calib_flag.png](doc%2Fcalib_flag.png)

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

Currently set `calib_flags=None`, `stereo_calib_flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL`, 
and the reprojection error of the calibration is small

Output log is as follows:
```shell
-- ====================== Stereo Calib Start ======================
-- ./data/calib_imgs/raw/../left\left_001.png ./data/calib_imgs/raw/../right\right_001.png
-- ./data/calib_imgs/raw/../left\left_002.png ./data/calib_imgs/raw/../right\right_002.png
-- ./data/calib_imgs/raw/../left\left_003.png ./data/calib_imgs/raw/../right\right_003.png
-- ./data/calib_imgs/raw/../left\left_004.png ./data/calib_imgs/raw/../right\right_004.png
-- ./data/calib_imgs/raw/../left\left_005.png ./data/calib_imgs/raw/../right\right_005.png
-- ./data/calib_imgs/raw/../left\left_006.png ./data/calib_imgs/raw/../right\right_006.png
-- ./data/calib_imgs/raw/../left\left_007.png ./data/calib_imgs/raw/../right\right_007.png
-- ./data/calib_imgs/raw/../left\left_008.png ./data/calib_imgs/raw/../right\right_008.png
-- ./data/calib_imgs/raw/../left\left_009.png ./data/calib_imgs/raw/../right\right_009.png
-- ./data/calib_imgs/raw/../left\left_010.png ./data/calib_imgs/raw/../right\right_010.png
-- ./data/calib_imgs/raw/../left\left_011.png ./data/calib_imgs/raw/../right\right_011.png
-- ./data/calib_imgs/raw/../left\left_012.png ./data/calib_imgs/raw/../right\right_012.png
-- ./data/calib_imgs/raw/../left\left_013.png ./data/calib_imgs/raw/../right\right_013.png
-- ./data/calib_imgs/raw/../left\left_014.png ./data/calib_imgs/raw/../right\right_014.png
-- ./data/calib_imgs/raw/../left\left_015.png ./data/calib_imgs/raw/../right\right_015.png
-- ./data/calib_imgs/raw/../left\left_016.png ./data/calib_imgs/raw/../right\right_016.png
-- ./data/calib_imgs/raw/../left\left_017.png ./data/calib_imgs/raw/../right\right_017.png
-- ./data/calib_imgs/raw/../left\left_018.png ./data/calib_imgs/raw/../right\right_018.png
-- ./data/calib_imgs/raw/../left\left_019.png ./data/calib_imgs/raw/../right\right_019.png
-- ./data/calib_imgs/raw/../left\left_020.png ./data/calib_imgs/raw/../right\right_020.png
-- left image reprojection error: 4.671930829291836, right image reprojection error: 4.859185223701936
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
-- stereo calibration reprojection error: 0.2238986518250022
-- ======================= Stereo Calib End =======================
-- F B cx cy doffs: 465.2104178592821, 69.82928990313776, 870.4762115478516, 221.603515625, -0.0
Save Json: D:\1_Code\3_Python\stereo_calib\data\calib_imgs\calib.json
Save yaml: D:\1_Code\3_Python\stereo_calib\data\calib_imgs\stereo_8.yaml
=================================================================================
=> Reprojection error is less than 0.5. Calibration successful. 
Please verify that the images in the 'rectify' directory have been correctly rectified.
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
