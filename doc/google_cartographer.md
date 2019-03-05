Inbetriebnahme Google Cartographer:

1. Einloggen.
2. Mehrere Terminal-Fenster öffnen.
3. Terminalfenster 1:
   . ros1_start.sh
   roscore
4. Terminalfenster 2:
  . ros1_start.sh
   cd ~/ros_catkin_ws
   source ./devel/setup.bash
5. Terminalfenster 3:
   roslaunch sick_scan sick_mrs_1xxx_cartographer.launch cloud_topic:=horizontal_laser_3d frame_id:=horizontal_vlp16_link
6. Terminalfenster 4:
   roslaunch sick_scan sick_tim_5xx.launch cloud_topic:=vertical_laser_3d frame_id:=vertical_vlp16_link hostname:=192.168.0.71
7. Terminalfenster 5:

   . ros1_start.sh
   cd ~/ros_cartographer_ws
   source ./install_isolated/setup.bash
   catkin_make_isolated

   roslaunch cartographer_ros live_demo_backpack_3d.launch


```
rosuser@ROS-NB:~/ros_catkin_ws$ rosbag info ~/Downloads/b3-2016-04-05-14-14-00.bag 
path:         /home/rosuser/Downloads/b3-2016-04-05-14-14-00.bag
version:      2.0
duration:     20:21s (1221s)
start:        Apr 05 2016 16:14:00.74 (1459865640.74)
end:          Apr 05 2016 16:34:21.97 (1459866861.97)
size:         9.1 GB
messages:     3986178
compression:  lz4 [29683/29683 chunks; 41.66%]
uncompressed: 21.8 GB @ 18.2 MB/s
compressed:    9.1 GB @  7.6 MB/s (41.66%)
types:        sensor_msgs/Imu         [6a62c6daae103f4ff57a132d6f95cec2]
              sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:       horizontal_laser_3d   1840435 msgs    : sensor_msgs/PointCloud2
              imu                    305308 msgs    : sensor_msgs/Imu        
              vertical_laser_3d     1840435 msgs    : sensor_msgs/PointCloud2
```







