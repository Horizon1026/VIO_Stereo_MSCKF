# ESKF_Estimator
A state estimator for robots using ESKF

# 开发计划
编写一个应用于单个IMU与多种不同传感器共同组成的硬件平台的多传感器融合定位代码，要求能够以 IMU 的更新频率来发布最新状态估计结果。
+ 完成 data loader，控制多种不同传感器的数据输入和初步处理
+ 完成 imu propagate 和序列化过程
+ 完成加入多目视觉观测的 VIO 过程
+ 完成加入 GPS 观测的多传感器融合过程
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
+ 已完成多目视觉 vio 的初始化逻辑，除了三角化和 PnP 之外，其余部分均已完善（）
+ 已完成多目视觉 update 部分的逻辑，除特征点选择、三角化、关键帧选择之外，其余部分均已完善（包括 propagator 重置起点，扩展状态协方差，构造完整量测方程，ESKF Update，误差状态更新到名义状态，以 Update 结果配置 propagator 起点并让他 repropagate）
+ 已完成三角测量和 PnP 模块相关接口，但三角测量评分机制尚待完善
+ TODO：上述两个模块完成后的系统整体调试与优化