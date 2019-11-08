# opencv343 x64
# Intel RealSense SDK 2.29.0 x64
汇总了关于opencv 3.4 版本的有关程序。
开发环境： 
	C++使用VS2015社区版 opencv3.4.3
	python3.6 opencv3.4.1

1.third-party\opencv343 
opencv的Release的库文件 x64

2. third-party\Intel RealSense SDK 2.0 
RealSense的库文件 x64



3. cascade
级联分类器的使用，使用opencv训练好的人脸识别模型，代码中包括一张图片的检测和打开0号摄像头的人脸检测
HARR与LBP的区别：
	1. HARR特征是浮点数计算
	2. LBP特征是整数计算
	3. LBP训练需要的样本数量比HARR大
	4. 同样的样本空间，HARR训练出来的数据检测结果要比LBP准确
	5. 扩大LBP的样本数据，训练结果可以跟HARR一样
	6. LBP的速度一般可以比HARR快几倍

4. matchTemplate
模板匹配

5. minMaxLoc
寻找图片中最大像素和最小像素的坐标值

6. ROI_save
图片区域保存

7.Trackbar
状态条的使用

8.Vec3b
设置三通道像素值

9.capture
通过realsense相机显示深度图像

10.arm
通过realsense摄像头获取小方块的位置，并打印小方块中心点的相机坐标点。

11.BackSub
背景切除操作，使用两种算法：MOG与KNN.