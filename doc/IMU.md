## IMU Support
The scanners of the MRS6xxx and MRS1xxx series will be optionally available with an IMU in 2019. 

## Activating IMU Messages
By setting the following config parameter in the launch file, the output of [imu messages](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) can be enabled with a compatible scanner. Currently the messages are published in the /Imu Topic.
```xml
<param name="imu_enable" type="bool" value="True" />

```
The imu Messages contain covariance matrices, these are currently determined from empirical values and are not measured specifically for each scanner. 
The laser scanner provides additional information (tick timestamp and confidence) to the Imu messages these can be activated by activating the [SickImu messages](../msg/SickImu.msg).

```xml
<param name="imu_enable_additional_info" type="bool" value="True" />

```
