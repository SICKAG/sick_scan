# sick_scan

This stack provides a ROS driver for the SICK lidar sensors mentioned in the following list.

## Supported Hardware

This driver should work with all of the following products.

ROS Device Driver for SICK lidar and radar sensors - supported scanner types:\

| **device name**    |  **part no.**                                                                                                                | **description**                                | **tested?**     |
|--------------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|:---------------:|
| TiM771S            | [1105052](https://www.sick.com/de/de/mess-und-detektionsloesungen/2d-lidar-sensoren/tim7xx/tim771s-2174104/p/p660929)                  | 1 layer max. range: 25 m, ang. resol. 0.33 [deg]| ✔ [stable]|

##  Start Node

Use the following command to start ROS node:

- For TiM771S:
```bash
rosrun sick_scan sick_generic_caller __name:=sick_tim_7xxS _hostname:=10.39.46.23
```

### Common parameters

- `scanner_type`
  Name of the used scanner. Usually this is also the name of the launch file. This entry is used to differentiate
  between the various scanner properties within the software code.

- `hostname`
  IP-address of the scanner (default: 192.168.0.1)

- `port`
  IP-port of the scanner (default: 2112)

- `min_ang`
  Start angle in [rad]

- `max_ang`
  End angle in [rad]

- `use_binary_protocol`
  Switch between SOPAS Binary and SOPAS ASCII protocol

- `intensity`
  Enable or disable transport of intensity values

- `intensity_resolution_16bit`
  If true, the intensity values is transferred as 16 bit value. If false, as 8 bit value.

- `min_intensity`
  If min_intensity > 0, all range values in a LaserScan message are set to infinity, if their intensity value is below min_intensity

- `frame_id`
  Frame id used for the published data


### Further useful parameters and features

- `timelimit`
  Timelimit in [sec] for max. wait time of incoming sensor reply

- `sw_pll_only_publish`
  If true, the internal Software PLL is forced to sync the scan generation time stamp to a system timestamp

- **Field monitoring**: The **TiM7xxS** families have [extended settings for field monitoring](./doc/field_monitoring_extensions.md).

## Sopas Mode
This driver supports both COLA-B (binary) and COLA-A (ASCII) communication with the laser scanner. Binary mode is activated by default. Since this mode generates less network traffic.
If the communication mode set in the scanner memory is different from that used by the driver, the scanner's communication mode is changed. This requires a restart of the TCP-IP connection, which can extend the start time by up to 30 seconds.
There are two ways to prevent this:
1. [Recommended] Set the communication mode with the SOPAS ET software to binary and save this setting in the scanner's EEPROM.
2. Use the parameter "use_binary_protocol" to overwrite the default settings of the driver.
3. Setting "use_binary_protocol" to "False" activates COLA-A and disables COLA-B (default)
