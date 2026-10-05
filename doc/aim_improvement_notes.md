# 自瞄改进学习笔记（对标强校开源）

> 2026-10：对本仓库做静态审查与改进时，调研了各强校 2023–2025 赛季的开源自瞄实现，
> 提炼可落地的技术点。本文记录：各项目的技术亮点、本仓库的差距、本次已落地项与未来方向。

## 一、调研对象与亮点

| 项目 | 亮点技术 |
|---|---|
| [rm_vision](https://github.com/chenjunnn/rm_vision)（原版框架，现由 [SCAU-RM-NAV](https://github.com/SCAU-RM-NAV/rm_vision) 等维护分支延续） | ROS2 规范化流水线；EKF 装甲板跟踪 + **装甲板切换（armor switching）状态机**；**独立火控模块**：按瞄准误差与跟踪状态给出开火许可，支持连发（burst）模式 |
| [Ericsii/rm_vision](https://github.com/Ericsii/rm_vision)（强化分支） | 整车 EKF（整车中心建模而非单装甲板）、弹道解算与硬件驱动分层 |
| [中南大学 FYT2024_vision](https://github.com/CSU-FYT-Vision/FYT2024_vision) | **BA 图优化位姿估计**（Ceres，替代单帧 PnP）；**亚像素灯条角点修正**（light_corner_corrector）；**整车运动模型 EKF**（motion_model）；火控：跟踪稳定确认（`overflow_count_ > transfer_thresh_` 才给 `fire_advice`）+ 显式 `yaw_diff/pitch_diff` 交给电控终判 |
| [华科 HUST_HeroAim_2024](https://github.com/HUSTLYRM/HUST_HeroAim_2024) | 英雄弹道标定流程与坐标系变换推导 |
| [北邮 rmdecis](https://github.com/cygnomatic/rmdecis) | SORT 目标跟踪、卡尔曼预测、弹道补偿 |
| [上交 CVRM aimer 文档](https://sjtu-robomaster-team.github.io/aimer_readme) | 反陀螺激活条件（>20rpm 才启用跟随，避免低速甩飞）；能量机关建模 |

## 二、对标结论：本仓库的差距

本仓库（配合 OpenRM）已具备：TensorRT 流水线、整车反陀螺模型（`antitop->getCenter/getPose`）、
EKF、哨兵双相机切换。**缺失且可独立落地的关键一项是火控开火许可**：

原实现在 `send_thread.cpp` 中

```cpp
fire = (fire && start_delay_flag && autoaim_flag && Data::auto_fire);
```

没有校验"解算指向与云台当前指向是否一致"——云台还在转动/抖动、
或解算值与机械指向有偏差时，只要其他条件满足就会发出开火指令。
这是 rm_vision / FYT / rmdecis 等所有参考实现里都有的标准环节。

另外，静态审查还发现反陀螺角速度滤波器的观测噪声 R 被误设为过程噪声 Q 的值
（相差 3 个数量级，见 3.4）。

## 三、本次已落地

### 3.1 火控误差门限 + 连续确认 + 滞回（`Control::send_thread()`）

配置见 `Config.json → Car`，三段式火控：

1. **进门限**：`yaw_err = normalize_angle(target_yaw - get_yaw())`、
   `pitch_err = target_pitch - get_pitch()`，双轴都小于
   `FireYawTolDeg` / `FirePitchTolDeg`（默认 1.0°）才可能开火；
2. **连续确认**（对标 FYT 的稳定计数思想）：需连续 `FireConfirmNum` 帧（默认 2）
   满足进门限，抑制云台抖动期的单帧误判；
3. **滞回退出**：确认后退出容差放宽 `FireExitRatio`（默认 1.5）倍——误差在
   门限边界抖动时 fire 不会高频震荡，避免点火毛刺；
4. **角度归一化**：yaw 差归一化到 [-π, π]，正确处理跨零；
5. 门限每拍计算、与 `auto_fire` 脉冲解耦（脉冲是瞬时置位，确认计数不能依赖它）；
6. 配置读取用 `json::value(key, default)`，**老配置文件缺键不崩溃**。

### 3.2 IPPE PnP 二义性时序消歧（`Pipeline::locater()`）

**问题**：IPPE 对平面矩形有两个镜像解。装甲板接近正对相机时两解重投影误差接近，
`cv::solvePnP` 只返回误差较小解——但此时两解的误差差在噪声量级内，
导致 armor yaw 随帧在 ±几十度间随机翻转（社区俗称"PnP 翻转"），污染反陀螺/EKF 输入。

**强校做法**：FYT 用 BA 图优化规避；rm_vision 靠 EKF 连续性吸收。

**本仓库的轻量实现**（不需要改 OpenRM、不引入新依赖）：

1. `cv::solvePnPGeneric(..., SOLVEPNP_IPPE, ..., sol_errors)` 取出全部解与各自重投影误差
   （OpenCV ≥4.0，本仓库要求 4.5.4 ✓）；
2. 对每个解做与原来相同的坐标链变换 `pnp → head → world`，得到各解的
   `armor_yaw_world` 与 `pose_world`；
3. 择优：同 (camera_id, armor_id) **首次观测取重投影误差最小解**；有历史后取
   **与上一帧 yaw 归一化距离最近的解**；
4. 历史表键 = (camera_id, armor_id)，数量上界 = 相机数 × ID 数，常量级内存。

预期效果：消除正对姿态附近的 yaw 翻转抖动，使 `antitop`/EKF 拿到的观测序列平稳，
小陀螺场景的预测精度直接受益。

### 3.3 反陀螺角速度滤波器 R 矩阵修复（`wrapper_car.cpp`）

原实现 `antitop_4_->setOmegaMatrixR(antitopOmegaQ[0])` 把**过程噪声 Q 表的值**
（OmegaQ=[1e1,…]，即 1e1）塞给了观测噪声 R，而配置的设计值是
`OmegaR=[1e-2]`——**差了 3 个数量级**（对照 balance 分支用的是 `BalanceOmegaR[0]`，
可确认是复制粘贴笔误）。R 被放大 1e3 倍意味着滤波器几乎不信观测，
小陀螺转速估计严重滞后，直接影响 antitop 开火时机与预测精度。已改用 `antitopOmegaR[0]`。

### 3.4 fourpoints 流水线颜色映射表用错（`fourpoints.cpp`）

原实现 `armor_color = (rm::ArmorColor)armor_class_map[yolo_rect.color_id]` ——
颜色查的是**类别表**。配置中 `ClassMap=[0,1,2,3,4,5,6,6,6]` 与
`ColorMap=[1,0,2,3]` 并不一致，九类模型下红/蓝直接颠倒或越界，
进而选错装甲板 3D 尺寸、PnP 距离与位姿全错。已改用 `armor_color_map`。
（对照 baseline 的 `pointer.cpp` 可确认两表应各司其职。）

### 3.5 其他修复

- `rune/tracker_thread.cpp`：与 baseline 相同的 `tp0` 未初始化入队、
  `1.0/getAvg()` 零除、`catch` 非 const 引用——统一修复；
- `fourpoints.cpp`：`catch` 非 const 引用修复；
- `CMakeLists.txt`：尊重外部 `-DCMAKE_BUILD_TYPE`（原实现无条件覆盖为 DEBUG，
  导致无法用 Release 出性能包），并补 `CMAKE_CXX_STANDARD_REQUIRED`。

### 3.6 顺手清理

`locater()` 中 `curr_size` 的冗余自赋值分支合并为三元式；未使用的 `pose_head`
等声明移除；标准库 include 归位。

## 四、未来方向（需实机验证或改 OpenRM，本次不动）

- **BA 位姿估计 + 亚像素灯条角点修正**（FYT）：收益比时序消歧更大，但需要改
  OpenRM 算法库或把角点估计本地化；
- **整车运动模型 EKF 的显式实现**（FYT motion_model / Ericsii）：本仓库的整车模型
  在 OpenRM 侧闭源细节中，建议后续按 motion_model.hpp 对照校准；
- **连发（burst）模式与弹舱管理**（rm_vision fire_control）；
- **combine 模式下 baseline/rune 寄存器共享的竞态消除**（见上一次提交的遗留记录）。
