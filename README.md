

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
mkdir -p catkin/src
cd catkin/src
catkin_init_workspace
git clone https://github.com/Xiao-Hu-Z/lidar_detection_track
cd ..
catkin_make
```

3.启动
```
roslaunch lidar_detection_track lidar_detection_track.launch
rviz -d rviz.rviz
```


