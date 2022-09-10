# ESKF_Estimator
A state estimator for robots using ESKF

# 开发计划
编写一个应用于单个IMU与多种不同传感器共同组成的硬件平台的多传感器融合定位代码，要求能够以IMU的更新频率来发布最新状态估计结果。
1、完成data loader，控制多种不同传感器的数据输入和初步处理
2、完成imu propagate和序列化过程
3、完成加入多目视觉观测的VIO过程
4、完成加入GPS观测的多传感器融合过程
5、考虑其他传感器

# 声明
代码基于个人兴趣开发，欢迎参与讨论，禁止商用
