# ESKF_Estimator
A state estimator for robots using ESKF

# 开发计划
编写一个应用于单个IMU与多种不同传感器共同组成的硬件平台的多传感器融合定位代码，要求能够以IMU的更新频率来发布最新状态估计结果。
+ 完成data loader，控制多种不同传感器的数据输入和初步处理
+ 完成imu propagate和序列化过程
+ 完成加入多目视觉观测的VIO过程
+ 完成加入GPS观测的多传感器融合过程
+ 考虑其他传感器

# 声明
+ 代码基于个人兴趣开发，欢迎参与讨论，禁止商用

# 进度记录
+ 已完成 data loader 部分，用于输入数据序列化管理（已初步验证）
+ 已完成 backend config 部分，用于相关参数配置，不依赖 opencv 或 yaml-cpp（已验证）
+ 已完成 feature/frame manager 部分，以及三种边缘化策略对应的数据管理操作，用于管理视觉特征点与关键帧（已验证）
+ 已完成 imu propagate queue 部分，属于 ESKF 的 propagate 部分（名义状态已经验证，误差状态及协方差待验证）
已将 IMU 状态从 18 维修改为 15 维，因为估计重力加速度会引入额外的误差
+ 已完成 attitude estimator 姿态解算求解器（已验证）
+ 已完成将 eskf_vio_backend 编译为 so 动态链接库，之后再通过链接库和引用头文件的方式来使用库（已验证）
为避免 vscode 不能识别多个 cmake 工程导致无法自动联想，影响代码开发进度，在 CMakeLists.txt 中增加一个分支，选择编译库后链接或者直接编译
+ TODO: Multi-View VIO 初始化（需要一个姿态估计器来估计姿态，需要至少一个双目系统来估计初始速度）
+ TODO: 多目视觉 update 部分，属于ESKF 的 update 部分（包括特征点选择、特征点三角化、量测方程构造、ESKF update、queue repropagate、关键帧选择、滑动窗口维度扩展或裁剪）