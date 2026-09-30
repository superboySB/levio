# LEVIO 研究材料摘要

论文与既有调研对话中直接关系到本次数据集验证的要点；整理日期：2026-09-30。

## 来源与保存方式

论文：Kühne 等，LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices，arXiv:2602.03294v1，2026-02-03。https://arxiv.org/abs/2602.03294

既有调研对话：分析 LEVIO 技术、标定和移植边界。https://chatgpt.com/share/6abb7607-6e20-83ec-b80e-76f04bd354a8

新数据集关联论文：TIO-Former，arXiv:2609.17198v2。https://arxiv.org/pdf/2609.17198v2

原论文 PDF 为 13,252,503 字节，官方固定版本链接：https://arxiv.org/pdf/2602.03294v1 。项目提供 research/download_paper.sh 下载并校验 SHA-256；原文不随仓库再分发。此版本标注的是授予 arXiv 的非独占分发许可。https://arxiv.org/licenses/nonexclusive-distrib/1.0/

## 论文中可直接比较的事实

完整 LEVIO 使用视觉特征、关键帧、IMU 预积分和局部优化；不使用回环检测。精度表基于 EuRoC Machine Hall 录制数据，并非本项目新数据集的实测真值。

表 V：标准分辨率下 MH01–MH05 的 ATE RMSE 依次为 0.96、0.92、3.69、8.33、3.38 m，简单平均约 3.46 m。表 II 的最优绝对误差配置在 40 m 子轨迹上的相对平移误差依次为 26.96%、28.86%、18.61%、48.77%、28.29%。

论文另报告 GAP9 开发板上的处理吞吐和功耗。20 FPS 不等于已经完成实物无人机的相机采集、IMU 同步和飞控闭环飞行，也不等于本项目 Docker 中 Python 版的实时性能。

论文描述 ORB 特征及 8 点 RANSAC；公开 Python main 使用 GFTT+BRIEF，直接调用 OpenCV findEssentialMat，缺少完整的低匹配数和空描述子保护。研究分支的 8 匹配保护门槛会改变这些失效帧的处理。论文未公布表 V 的完整参数或 RPG 对齐配置；EuRoC 输出一致不等于复现论文数值。

## 既有对话对本实验的关键提示

Python 版默认读 EuRoC 的 /cam0/image_raw、/imu0，沿用 EuRoC 的相机内参、畸变和相机/IMU 外参；新相机与新 IMU 必须替换对应标定，单目输入不免除标定。

代码中的 IMU 测量坐标变换只用外参旋转；保存 4×4 平移并不意味着已经加入 IMU 杠杆臂补偿。视觉惯性初始化也不会自动估计相机内参、传感器安装外参或固定时间偏移。

Python 版是离线算法实验入口；论文的 EuRoC 精度和 GAP9 性能属于不同的测量条件。既有对话是二手分析，以上要点已按当前仓库代码和论文重新核对。

TIO-Former 第 IV-A 节以外部动捕取得真值，并将端点漂移率定义为端点位置误差除以同窗口真值路程。其图 5 对比从共同初位姿积分的开放环轨迹。这能说明本实验为什么固定起点，但公开 odom_dataset bag 中 /fusion_odometry/lazy_point_odom 的真值来源不能据此推定为动捕或 LIO。

TIO-Former 保留测试序列的真值路程标准化为 4–7 m。本报告另给出预定的 5 m 参考路程窗，与 10 s、30 s 非重叠时间窗并列，逐窗仅以第一对完整位姿确定 SE(3) 起点。窗口边界只改变离线评价坐标，不重启在线 VIO；由于传感器、参考来源和协议不同，5 m 窗的数字不能与 TIO-Former 直接排名。

## 与 research 分支的对应关系

本实验从逐段 bag 的 camera_info 读取 D435i 彩色内参；用已发布的机体至左红外模板外参和 bag 内 RealSense TF 推得机体至彩色相机变换。输入 IMU 是飞控 /mavros/imu/data_raw，不是 D435i 自带 IMU。

所选 18 段的时间质量、初始化状态和轨迹比较以项目根目录 note.md 为准。/fusion_odometry/lazy_point_odom 仅作记录里程计参考：公开数据未证明它由 LIO 生成，也未给出独立地面真值误差。
