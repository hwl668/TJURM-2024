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

## 三、本次已落地

`Control::send_thread()` 增加火控误差门限（配置驱动，见 `Config.json → Car`）：

1. **双轴误差门限**：`yaw_err = normalize_angle(target_yaw - get_yaw())`、
   `pitch_err = target_pitch - get_pitch()`，都在容差内才允许开火
   （`FireYawTolDeg` / `FirePitchTolDeg`，默认 1.0°）。
2. **连续确认**（对标 FYT 的稳定计数思想）：需连续 `FireConfirmNum` 帧（默认 2）
   满足门限，抑制云台抖动期的单帧误判。
3. **角度归一化**：yaw 差归一化到 [-π, π]，正确处理跨零。
4. 误差门限每拍计算、与 `auto_fire` 脉冲解耦（脉冲是瞬时置位，确认计数不能依赖它）。
5. 配置读取用 `json::value(key, default)`，**老配置文件缺键不崩溃**。

## 四、未来方向（需实机验证或改 OpenRM，本次不动）

- **BA 位姿估计 + 亚像素灯条角点修正**（FYT）：收益明确，但需要改 OpenRM 算法库；
- **整车运动模型 EKF 的显式实现**（FYT motion_model / Ericsii）：本仓库的整车模型
  在 OpenRM 侧闭源细节中，建议后续按 motion_model.hpp 对照校准；
- **连发（burst）模式与弹舱管理**（rm_vision fire_control）；
- **combine 模式下 baseline/rune 寄存器共享的竞态消除**（见上一次提交的遗留记录）。
