# sick_scan
sick_scan
================

This stack provides a ROS driver for the SICK series of laser scanners mentioned in the following list.
The SICK MRS6124 is a multi-layer, multi-echo 3D laser scanner that is geared
towards rough outdoor environments. 

Supported Hardware
------------------

This driver should work with all of the following products. However, this driver is brand new (Feb 2018) 
and tested working on the MRS6124. The migration with the already publish sick_scan-driver (see github.com/SICKAG/sick_scan ) is in preparation
and will be finished until end of March 2018.

ROS Device Driver for Sick Laserscanner - supported scanner types: 
[MRS6124] (unpublished)
[MRS1104](https://www.sick.com/media/docs/1/51/551/quickstart_MRS1000_de_IM0073551.PDF),
[TiM551](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF),
[TiM561](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF),
[TiM571](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF) 

Use the following command to start ROS node:

For MRS6124:
>> roslaunch sick_scan sick_mrs_6xxx.launch

For MRS1104:
>> roslaunch sick_scan sick_mrs_1xxx.launch

For TiM551, TiM561, TiM571:
>> roslaunch sick_scan sick_tim_5xx.launch

# Troubleshooting 

1. Check Scanner IP in the launch file. 
2. Check Ethernet connection to scanner with a ping. 
3. View node startup output wether the IP connection could be established 
4. Check the scanner status using the LEDs on the device. The LED codes are described in the above mentioned operation manuals.
5. Further testing and troubleshooting informations can found in the file test/readme_testplan.txt

# SUPPORT
 
* In case of technical support please open a new issue. 
* In case of application support please use [https://supportportal.sick.com ](https://supportportal.sick.com).


Installation
------------

In the following instructions, replace `<rosdistro>` with the name of your ROS distro (e.g., `indigo`).

### From binaries

The driver has not been released yet. But once that happens, you can install it directly by typing:

~~`sudo apt-get install ros-<rosdistro>-sick_scan`~~

### From source

```bash
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone https://github.com/michael1309/sick_scan.git
cd ..
catkin_make
source ~/ros_catkin_ws/install/setup.bash
```

Quick Start
-----------

```bash
roslaunch sick_scan sick_mrs6xxx.launch
rosrun rviz rviz
publish to point clound
```


------------------------------------------------------------------------

![SICK Logo](img/Logo_SICK_AG_2009.png "SICK Logo")

