# GitHub算法参考记录（2026-09-24）

## 采用的原则

1. `chenjunnn/rm_auto_aim`
   - 地址：https://github.com/chenjunnn/rm_auto_aim
   - 其跟踪器明确区分 LOST、DETECTING、TRACKING、TEMP_LOST。
   - 当前项目借鉴这一生命周期原则：暂时丢失轨迹保留以便重新关联，但只有当前帧真正观测到的敌方轨迹可以产生射击解。

2. `ifzhang/ByteTrack`
   - 地址：https://github.com/ifzhang/ByteTrack
   - 它分别维护 tracked、lost、removed 集合，并只输出激活的 tracked 轨迹；lost轨迹用于重新关联。
   - 当前项目没有引入其神经网络、IoU匹配或依赖，只采用“保留轨迹不等于允许射击”的原则。

3. `cygnomatic/rmdecis`
   - 地址：https://github.com/cygnomatic/rmdecis
   - 提供SORT、卡尔曼滤波、弹道补偿的模块化参考，但仓库自己声明尚未实战测试，因此未直接移植代码或参数。

## 未采用的部分

- 三维PnP、机器人姿态EKF、装甲编号分类不适用于当前二维Godot考核。
- ByteTrack的双阈值检测依赖真实置信度；当前传统颜色检测的confidence固定为1，直接照搬没有意义。
- 此前短窗口速度估计实测退步；本次用真实轨迹复核后仍未采用。

## 本地数据验证

以下仅为早期离线分析记录：使用另一套检测/轨迹提取脚本，并非当前C++端到端回放。不能据此宣称当前模型最优或给出实战性能保证；原分析数据不随源码包发布。

使用 `work/video_analysis/tracks.json` 中无操作超大杯录像提取的30条轨迹比较不同历史窗口。在0.8秒预测跨度上：

- 1.0秒匀加速窗口：中位误差2.54像素，90%误差10.26像素。
- 1.2秒匀加速窗口：中位误差2.01像素，90%误差7.82像素。
- 0.5秒和0.8秒窗口均更差。

因此只把历史窗口从1.0秒延长到1.2秒。
