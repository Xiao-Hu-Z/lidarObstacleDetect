

1.安装相应的 ros 依赖包
```
sudo apt-get install ros-melodic-jsk-rviz-plugins
sudo apt-get install ros-melodic-jsk-recognition-msgs
sudo apt-get install ros-melodic-autoware-msgs
sudo apt-get install ros-melodic-visualization-msgs
```

2.clone代码，编译：
建立一个工作空间，比如：

```
mkdir -p catkin_ws/src
cd catkin/src
catkin_init_workspace

# 将解压后的代码放入src目录下
# 编译
catkin_make

# 加入环境变量
vim ~/.bashrc
# 加入以下下代码
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
```

3.启动
```
roslaunch lidar_obstacle_detection lidar_obstacle_detection.launch
rviz -d rviz.rviz
```


