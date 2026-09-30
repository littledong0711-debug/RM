# 视觉组入队考核：二维目标跟踪与自动瞄准

本项目通过ROS 2接收考核游戏画面，识别敌我装甲板，预测目标运动并计算拦截角度，通过串口控制游戏发射装置。采用C++17、OpenCV与面向对象设计。

## 1. 环境与准备

已验证环境：Ubuntu 22.04 / WSL2、ROS 2 Humble。游戏程序由考核方提供，不包含在仓库中。
需先安装ROS 2 Humble，并确保以下依赖可用：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions libopencv-dev ros-humble-cv-bridge ros-humble-ros2launch ros-humble-ament-cmake-test
```

以上命令用于补充构建依赖，不代替ROS本体及其软件源的安装。

## 2. 编译与测试

下载或克隆本仓库，在项目根目录打开Linux/WSL终端：

```bash
bash scripts/build_and_test.sh
```

脚本会加载ROS环境、编译并运行3项测试。首次编译需要一定时间，不依赖预先生成的build或install目录。

## 3. 运行

1. 启动考核游戏，选择难度，查看当前Slave串口，例如/dev/pts/6。
2. 在本项目根目录的另一个终端执行：

```bash
bash run_test.sh 6 large
```

3. 开始游戏。程序确认炮台颜色后自动跟踪异色目标并射击。
4. 本局结束后按Ctrl+C。下一局重新查看串口并重启节点，即使端口编号未变。

6替换成当前端口编号，也支持完整路径/dev/pts/6。
难度标签为middle、large、super，仅用于日志命名；实际难度在游戏中选择。
日志保存在diagnostics/repeated_tests，记录参数与程序哈希。

如需直接运行ROS节点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run armor_aim armor_aim_node --ros-args --params-file src/armor_aim/config/params.yaml -p serial_port:=/dev/pts/6
```

## 4. 实现思路

图像解码 → 炮台颜色确认 → 敌我分别检测和跟踪 → 求解拦截位置 → 友军弹道检查 → 转向 → 下一帧复核后开火。

| 模块 | 主要职责 |
| --- | --- |
| ArmorDetector | 颜色差分提取灯条，配对装甲框，定位炮台 |
| TargetTracker | 最近邻关联、Kalman滤波、历史窗口加速度拟合 |
| Ballistics | 根据速度、加速度、控制补偿和弹速求最早拦截解 |
| 友军保护 | 检查子弹完整离屏轨迹与友军预测范围是否相交 |
| SerialPort | 管理串口，发送转向及开火指令 |
| ArmorAimNode | ROS图像订阅、选敌、两帧开火复核 |

[详细代码导读](docs/CODE_GUIDE.md) · [主程序](src/armor_aim/src/armor_aim_node.cpp) · [参数配置](src/armor_aim/config/params.yaml)

## 5. 接口与参数

- 输入话题：/image_raw，游戏图像1152×648。
- 转向指令：0x01 + 小端float32角度，0度向右；开火指令：0x02。
- 当前子弹速度：600像素/秒。
- 当前敌方额外预测补偿：0.03秒；友军检查补偿：0.12秒。
- 当前采用折中友军保护，保留匀速与匀加速风险检查。
- 参数修改后需重新启动节点；补偿值是经验设置，不是实测硬件延迟。

## 6. 测试结果与局限

已完成初步游戏实测，验证了目标识别、跟踪、预测瞄准及自动射击的基本流程。
不同随机种子生成的场景存在差异，实际得分会有波动，不能保证每局达到相同或稳定的高分。历史单局成绩不作为稳定性能承诺。

自动化测试共3项：基础算法与伪串口、友军弹道交叉检查、12张实录图像回归，最近一次本地执行全部通过。测试数据随源码提供，不依赖本地日志目录。

仍存在随机变速预测误差及偶发误伤，尚无独立灰块障碍避让，也未识别剩余血量。建议使用多个固定种子重复测试，结合平均分、最低分和误伤次数评估；得分波动不能全部归因于随机性。

本项目采用的游戏数量为敌方20、友方10，原始满分100。敌方三次命中+1/+1/+3，友方三次命中-5/-5/-10；敌方逃逸-1。净降1分不一定是误伤。难度加权为中杯0.5、大杯0.75、超大杯1.0。

## 7. 常见问题

| 现象 | 排查方式 |
| --- | --- |
| 未找到安装文件或ROS包 | 先在项目根目录运行构建脚本 |
| 串口等待/没有动作 | 核对本局Slave端口、游戏是否开始、/image_raw是否有数据 |
| 提示已有节点 | 在旧终端Ctrl+C，避免两个节点同时控制 |
| 测试缺少图片 | 检查src/armor_aim/test/data/recorded_frames是否完整下载 |

## 8. 目录

```text
README.md               考核说明及使用步骤
run_test.sh             游戏运行入口
scripts/               构建与测试入口
docs/                  算法导读与参考来源
src/armor_aim/
  config/              参数
  include/armor_aim/   类型与算法接口
  src/                 C++实现
  launch/              ROS启动描述
  test/                测试代码和固定图片
```

开发过程中使用AI辅助编写、整理及迭代；算法参考见[参考说明](docs/REFERENCES.md)。
