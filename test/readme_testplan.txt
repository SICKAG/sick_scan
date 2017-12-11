Testing plan

Requirement:
1. Linux (tested with Ubuntu 16.02)
2. ROS environment (tested with ros indigo)
3. SICK scanner with Ethernet connection
4. Checked out and built ROS repo

Proposed Test Sequence

1. Switch on the scanner
2. Launch the file in the editor.
3. Check scanner IP address availability: ping aa.bb.cc.dd
    Example for use in the local network: ping 192.168.0.2
4. If the scanner is pingable, set the correct Linux environment for ROS:
    a) Change to the ROS-workspace directory:
        cd <ws>
        For example: cd ~ / catkin_ws
    b) Start setup script to set up the ROS environment:
        Input: source ./devel/setup.bash
5. Start ROS nodes of the selected scanner type:
    Command: roslaunch sick_scan <id> .launch
    The following scanner types are currently supported:
    MRS1104
    TiM 571
    The launch files are named as follows:
    sick_tim_5xx.launch for TiM571
    sick_mrs_1xxx.launch for MRS1104
    Example (will be used in the following).
    roslaunch sick_sccan mrs_1xxx.launch

6. To check the node at the console level, the following tests are performed in another terminal window:
a) rosparam list [ENTER]: output all ROS parameters
b) rosparam list / mrs_1xxx / * Output of the parameters for the MRS1104
c) rostopic hz / scan: Check the scan rate output rate
d) rostopic hz / cloud: Check the Pointcloud2 output rate

7. Check the multi-target ability:
    a) Cancel roslaunch with CTRL-C (or alternatively with kill <process>)
	b) Get the BIT mask for the goals:
       Example: rosparam get / mrs_1xxx / active_echos
    b) Set the BIT mask for the goals:
	c) rosparam set /mrs_1xxx 7 (bit-codiere Auswahl - hier: DIST1, DIST2, DIST3)	
c) Check the scan rate for the scan output
rostopic hz / scan (must now be 50 Hz * (number of targets set)

Repeat steps a) to d) for the following
BIT masks: 1, 4, 2, 3

8. Check the Pointcloud2 output:
    a) Start visualization tool from ROS:
       rosrun rviz rviz

    b) Subscribe to scan messages via the following steps (see also Chapter 4 at: http://wiki.ros.org/rviz/UserGuide)
       Add -> Laserscan
	   Add -> Pointcloud2

    c) Check the visualization
	
9. Angle specification:
The angle range can also be set via min_ang and max_ang.
The setting is in radians. The test will otherwise follow the logic above. Recommended angle range test: +/- 45 °:
min_ang: -0.785398
max_ang: +0.785398

The representation in rviz must then be limited to this area.


   
   
      
	  
		