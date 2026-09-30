# 参考说明

[返回项目说明](../README.md)

- [rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim)：参考目标跟踪生命周期的划分，区分当前可见目标与暂时丢失轨迹。
- [ByteTrack](https://github.com/ifzhang/ByteTrack)：参考“保留轨迹用于重新关联，不等于允许对其射击”的原则；未引入其神经网络或双阈值匹配实现。
- [rmdecis](https://github.com/cygnomatic/rmdecis)：调研过其模块化方案，未直接移植代码或参数。

本项目针对二维游戏使用传统颜色检测、Kalman滤波与拦截求解，不使用三维PnP或装甲编号分类。
上述项目仅作为设计参考，不作为本项目实战得分或稳定性的证明。
