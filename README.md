# opencv343 x64
# Intel RealSense SDK 2.29.0 x64
汇总了关于opencv 3.4 版本的有关程序。
开发环境： 
	C++使用VS2015社区版 opencv3.4.3
	python3.6 opencv3.4.1

# 第三方库
1. third-party\opencv343 
opencv的Release的库文件 x64

2. third-party\Intel RealSense SDK 2.0 
RealSense的库文件 x64

3. eigen337
矩阵运算的c++库

4. PCL
下载PCL-1.8.1-AllInOne-msvc2015-win64.exe进行安装，然后导入PCL.props配置文件。


# 示例代码

## arm
realsense相机结合机械臂实现物体的抓取动作，功能包括： 相机标定、手眼标定以及物体识别等

## BackSub
对静止的背景进行切除操作，算法有：MOG与KNN.

## calibrateHandEye
opencv4.1.1实现eye-in-hand，计算出R_cam2gripper和t_cam2gripper。

## Calibration
相机内参标定，通过opencv实现以和realsense SDK两种方式。

## capture
通过realsense相机显示深度图像

## cascade
级联分类器的使用，使用opencv训练好的人脸识别模型，代码中包括一张图片的检测和打开0号摄像头的人脸检测
HARR与LBP的区别：
	(1). HARR特征是浮点数计算
	(2). LBP特征是整数计算
	(3). LBP训练需要的样本数量比HARR大
	(4). 同样的样本空间，HARR训练出来的数据检测结果要比LBP准确
	(5). 扩大LBP的样本数据，训练结果可以跟HARR一样
	(6). LBP的速度一般可以比HARR快几倍

## eigen_geometry
旋转矩阵 -- 欧拉角 -- 角轴
其中包括了opencv实现的欧拉角到旋转矩阵的转换功能。

## eigen_sample
Eigen的矩阵运算简单使用

## eye-hand
eye-in-hand实现，网络资源,基本功能是首先进行摄像头的标定，然后通过tsai算法计算出Mx。

## file_opers
opencv中文件的读写操作

## inRange
使用inRange函数检测视频中蓝色的物体，capture.open(2)设置打开第几个摄像头。

## mat
opencv中Mat的使用

## matchTemplate
模板匹配，使用模板文件匹配图片

## minMaxLoc
寻找图片中最大像素和最小像素的坐标值

## PoseEstimation
通过d415相机抓取深度图并生成点云，经过滤波、平面分离以及聚类，最后使用PCA计算位姿。

## ROI_save
图片区域保存

## shapeMatch
基于模板制作不同角度的2D位姿，通过匹配姿势实现位姿估计。

## Trackbar
状态条的使用

## Vec3b
设置三通道像素值


## pcl_CapturePcd
从realsense相机中读取深度和BGR信息，保存到PCD文件。


## pcl_PLY2PCD
读取ply文件并转成pcd文件，随后读取pcd并显示

## pcl-depthShowPc
从深度相机或者bag文件读取一帧深度图片，将其转换成点云并显示。

