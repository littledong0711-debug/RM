# 代码导读

[返回首页](../README.md) · [配置](../src/armor_aim/config/params.yaml)

```text
/image_raw
  → 图像解码与炮台颜色确认
  → 敌方/友方分别检测、分别跟踪
  → 选择敌方并解算拦截点
  → 检查友军是否穿过完整弹道
  → 发转向命令
  → 下一帧复核实际待发角度与目标
  → 再检查友军，通过后开火
```

## 按这个顺序阅读

1. [types.hpp](../src/armor_aim/include/armor_aim/types.hpp)：检测结果、位置/速度/加速度、累计与连续观测计数。
2. [armor_aim_node.cpp](../src/armor_aim/src/armor_aim_node.cpp)：ROS订阅、模块协调、两帧开火确认。
3. [armor_detector.cpp](../src/armor_aim/src/armor_detector.cpp)：彩色灯条配对；距离变换定位炮台转轴。
4. [target_tracker.cpp](../src/armor_aim/src/target_tracker.cpp)：最近邻关联、Kalman状态[x,y,vx,vy]、历史二次拟合。
5. [ballistics.cpp](../src/armor_aim/src/ballistics.cpp)：目标未来距离等于弹速乘飞行时间，选择最早可达解。
6. [game_aim.hpp](../src/armor_aim/include/armor_aim/game_aim.hpp)：水平运动约束、屏幕外解剔除、选敌评分。
7. [fire_guard.hpp](../src/armor_aim/include/armor_aim/fire_guard.hpp)：完整离屏轨迹的友军碰撞检查。
8. [serial_port.cpp](../src/armor_aim/src/serial_port.cpp)：RAII管理连接、处理部分写入、发送协议。

## 容易误解的地方

- 游戏四通道缓冲错误标成rgb8，特殊分支按RGBA解码；普通消息走cv_bridge。
- 图像Y轴向下，串口角度Y轴向上，atan2中需翻转Y。
- 0.03秒只是额外补偿，不能替代距离/600得到的飞行时间。
- hit_streak是累计观测数；consecutive_*才会在漏检后重计。
- 拟合加速度为0可能表示拟合失败回退，不一定表示真实匀速。
- 保存丢失轨迹用于关联，不表示允许向它开火；友军漏检仍参与保护。
- 日志1秒限频不等于发射1秒一次。

## 验证边界

test/self_test.cpp检查基础算法和伪串口；fire_guard_test.cpp检查给定运动模型下的碰撞逻辑；recorded_frames_test.cpp检查固定实录图像。
它们不能证明随机游戏中不会误伤。目前未识别血量，也没有独立灰块障碍避让。
置信度目前固定为1，不是统计概率；不要直接用来计算预期得分。
