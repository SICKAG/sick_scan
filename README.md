# sick_scan
ROS Device Driver for Sick Laserscanner - supported scanner types: 
[MRS1104](https://www.sick.com/media/docs/1/51/551/quickstart_MRS1000_de_IM0073551.PDF),
[TiM551](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF),
[TiM561](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF),
[TiM571](https://www.sick.com/media/docs/9/29/229/Operating_instructions_TiM55x_TiM56x_TiM57x_de_IM0051229.PDF) 
Use the following command to start ROS node:

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




