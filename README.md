# opencv343
汇总了关于opencv 3.4 版本的有关程序。
开发环境： 
	C++使用VS2015社区版 opencv3.4.3
	python3.6 opencv3.4.1

1.opencv343 
opencv的Release的库文件

2. cascade
级联分类器的使用，使用opencv训练好的人脸识别模型，代码中包括一张图片的检测和打开0号摄像头的人脸检测
HARR与LBP的区别：
1. HARR特征是浮点数计算
2. LBP特征是整数计算
3. LBP训练需要的样本数量比HARR大
4. 同样的样本空间，HARR训练出来的数据检测结果要比LBP准确
5. 扩大LBP的样本数据，训练结果可以跟HARR一样
6. LBP的速度一般可以比HARR快几倍

