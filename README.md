# Armor Aim · 二维游戏视觉跟踪

基于 **ROS 2 Humble / C++17 / OpenCV** 的入队考核项目：读取游戏图像，识别敌我、跟踪运动、计算拦截角度，并通过串口控制游戏中的发射装置。

当前仅保留 **balanced折中保护版**，敌方额外预测补偿 **0.03秒**。本项目由使用者与AI辅助迭代开发；测试通过不代表已解决全部误伤或随机变速问题。

[快速开始](docs/QUICKSTART.md) · [代码导读](docs/CODE_GUIDE.md) · [游戏规则](docs/RULES.md) · [参考来源](docs/REFERENCES.md) · [上传前检查](docs/GITHUB_CHECKLIST.md)

## 快速使用

先安装ROS 2 Humble及构建依赖，详见快速开始。下载后在项目根目录打开WSL终端：

```bash
bash scripts/build_and_test.sh
bash run_test.sh 6 large
```

6换成游戏当前Slave接口号，large可换middle或super。难度参数只标记日志，游戏难度在界面选择。
每局结束Ctrl+C，下一局重新查看端口并重启。

## 文件导航

使用VS Code时打开[armor_aim.code-workspace](armor_aim.code-workspace)，可隐藏本地缓存和日志，专注源码与文档。隐藏不等于删除；文件仍保留在磁盘。

```text
.
├── README.md                 项目首页
├── run_test.sh               游戏测试入口
├── docs/                     安装、规则、代码导读和提交清单
├── scripts/
│   └── build_and_test.sh     一键构建与测试
├── src/armor_aim/
│   ├── config/params.yaml   当前唯一参数配置
│   ├── include/armor_aim/   类型及算法接口
│   ├── src/                 C++实现
│   ├── launch/              ROS启动描述
│   └── test/                测试代码与固定输入图片
└── tools/                    采集、检查和源码打包工具
```

本地build/install/log/diagnostics/docs/local/dist不进入GitHub；构建产物可再生成，实测日志与本地记录仍保留。

## 工作流程

图像解码 → 确认炮台颜色 → 敌我分别跟踪 → 求拦截解 → 友军弹道检查 → 转向 → 下一帧复核后发射。

- 颜色灯条配对检测，Kalman滤波和历史二次拟合估计运动。
- 预测同时考虑目标运动、子弹飞行时间和额外延迟。
- 敌方漏检不射击；友军漏检仍保留保护轨迹。
- 检查完整离屏弹道，不只检查预定命中位置。

[查看主流程](src/armor_aim/src/armor_aim_node.cpp) · [查看参数](src/armor_aim/config/params.yaml) · [查看测试](src/armor_aim/test)

## 验证与限制

共有3项测试：基础算法/伪串口、友军碰撞检查、12张实录图像回归。固定图片随源码提供，测试输出写入构建目录。
本地环境为Ubuntu 22.04 / WSL2，ROS 2 Humble；不保证其他环境无需调整。

### 初步实测说明

项目已进行初步游戏实测，完成了目标识别、运动跟踪、预测瞄准及自动射击的基本流程验证。由于不同随机种子生成的游戏场景存在差异，实际得分会有波动，目前不能保证每局都达到相同或稳定的高分。历史单局成绩仅作为测试记录，不作为稳定性能承诺。

得分稳定性仍需通过多个固定种子、多轮重复测试进一步评估，并结合平均分、最低分及误伤次数综合判断。随机场景是影响成绩的因素之一，现阶段也不能排除检测、跟踪和预测误差的影响。

仍可能有误伤与变速预测偏差；没有独立灰块障碍避让，也未识别目标剩余血量。
不要用历史单局最高分代表稳定性能，应以同种子、多局统计评估。

## 发布前

维护者身份和开源许可尚需使用者确认，详见[上传前检查](docs/GITHUB_CHECKLIST.md)。
游戏程序、任务书和完整录屏不包含在本项目。不要将日志中的本机路径和个人信息一起公开。
