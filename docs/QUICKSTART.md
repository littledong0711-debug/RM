# 安装、编译与运行

[返回首页](../README.md)

## 环境

本项目在Ubuntu 22.04 / WSL2、ROS 2 Humble、C++17、OpenCV环境验证。
需安装ROS 2 Humble以及colcon、cv_bridge、OpenCV开发库；游戏程序由考核方另行提供，不包含在仓库。
已有ROS环境可安装缺少的构建依赖：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions libopencv-dev ros-humble-cv-bridge ros-humble-ros2launch ros-humble-ament-cmake-test
```

若尚未安装ROS，请先按ROS官方Humble安装文档配置系统软件源；上面命令不代替ROS安装。

## 下载后第一次运行

在解压或克隆得到的项目根目录打开WSL终端：

```bash
bash scripts/build_and_test.sh
```

正常应包含3项测试且无失败。脚本无需chmod，路径不依赖原作者用户名。

## 开始游戏

1. 单独启动考核游戏，选择难度，查看本局Slave串口，如/dev/pts/6。
2. 在本项目目录执行下面命令，打开录屏，再开始本局。

```bash
bash run_test.sh 6 large
```

难度标签middle/large/super仅用于日志命名，实际难度须在游戏界面选择。
每局结束Ctrl+C；重新开局后读取最新端口，即使接口没变也重新启动节点。
兼容第三参数balanced，不再保留其他实验档。

## 常见问题

| 现象 | 检查 |
| --- | --- |
| Package not found / 缺少install/setup.bash | 在项目根目录重新执行构建脚本 |
| Waiting for serial port | 使用游戏当前显示的Slave接口，不能沿用旧编号 |
| 串口已打开但无动作 | 游戏是否开始，/image_raw是否有画面，炮台颜色是否确认 |
| 提示已有节点运行 | 在旧终端Ctrl+C，不要同时运行两个节点 |
| 测试缺少图片 | 检查src/armor_aim/test/data/recorded_frames是否完整下载 |

只读检查图像：先source /opt/ros/humble/setup.bash，再运行ros2 topic hz /image_raw。
不要把启动命令粘贴进正在运行的游戏进程终端。

## 参数与记录

参数统一在[src/armor_aim/config/params.yaml](../src/armor_aim/config/params.yaml)。修改参数后重启节点。
日志位于diagnostics/repeated_tests，不提交GitHub；记录程序/配置哈希，可区别测试版本。
当前敌方补偿0.03秒，友军检查0.12秒；这是经验设置，不是已测得的硬件延迟。
