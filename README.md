# cyberdog_sim

基于cyberdog2的gazebo仿真实现

实现效果如下

地形适应

https://github.com/WLinZhen/cyberdog_sim/assets/144916746/376e69a2-d141-49e4-975d-56219c551378


基于地形适应的落足点规划

https://github.com/WLinZhen/cyberdog_sim/assets/144916746/fbeb1b39-5bef-408b-bcd7-a6084f8f023a

部署环境

1. ubuntu20.04通过ros2一键安装命令安装ros2-galactic

```
wget http://fishros.com/install -O fishros && . fishros
```

2. 安装gazebo11

```
sudo apt-get install gazebo11-*

sudo apt-get install ros-galactic-gazebo-ros-*

sudo apt install ros-galactic-xacro
```

编译项目

在cyberdog_sim文件夹下打开终端，确保当前在cyberdog_sim文件夹下

```
colcon build
```

使用方法

1. 启动仿真

```
ros2 launch sim_base sim.launch.py
```

2. 启动可视化程序

```
ros2 launch sim_base rviz.launch.py
```

3. 启动控制器 --> 使用方法请阅读 cyberdog_sim/src/cyber_guide/robot/src/rt/rt_keyboard.cpp

```
ros2 run cyber_guide cyber_guide
```