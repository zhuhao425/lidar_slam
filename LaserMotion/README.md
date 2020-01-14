# 里程计去激光雷达运动畸变模块

需要首先编译安装champion_nav_msgs，按照champion_nav_msgs的readme文件执行即可，注意根据自己ubuntu的不同版本做修改。

## 运行
```bash
$> catkin_make
$> source devel/setup.bash
$> roslaunch LaserUndistortion LaserUndistortion.launch

$> rosbag play –clock <your file>.bag
```
如果一切正常，则会看到pcl的可视化界面，当可视化界面中存在数据的时候，按R键即可看到结果。红色为畸变矫正前，绿色为畸变矫正后。

