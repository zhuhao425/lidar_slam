# odom_ws

## 说明
使用直接线性方法来用激光里程计对机器人的轮式里程计进行校正，需自己准备数据。

## 运行
```bash
$> catkin_make
$> source devel/setup.bash
$> roslaunch calib_odom odomCalib.launch

$>rviz
Add选项卡中增加三条Path消息。一条订阅的topic为：odom_path_pub_;一条订阅的topic为:scan_path_pub_；最后一条为:calib_path_pub_。分别选择不同的颜色。

$>rosbag play –clock odom.bag

$>rostopic pub /calib_flag std_msgs/Empty "{}"
```
