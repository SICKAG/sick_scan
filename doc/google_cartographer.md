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





