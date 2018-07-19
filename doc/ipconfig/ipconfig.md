#IP addresses configuration

##Starting with a new scanner
The scanner is delivered with a standard IP address, to read or change it the [SICK SOPAS ET](https://www.sick.com/de/de/sopas-engineering-tool-2018/p/p367244) for windows can be used.
When the tool is started, a search is performed which lists all scanners available in the network. 
![SOPAS start](./scanner_found.PNG "SOPAS start")
Double-click to select the scanner for the project
![SOPAS select](./scanner_added.PNG "SOPAS select")
double click on the scanner icon to open the configuration menu of the scanner
Select here the network configuration menu and set the parameters by clicking on the save icon (red arrow)
![SOPAS save network](./set_config.PNG "SOPAS save network")
To ensure that the settings are retained even after a Powercycle, they must be stored in the eeprom. To do this, click on the eeprom icon and confirm the save. 
![SOPAS save eeprom](./save_permanent.PNG "SOPAS save eeprom")
To test the settings under Windows use the comandos ```ipconfig``` and ```ping``` in the ```cmd.exe```. Make sure that the scanner and host pc have different ip addresses e.g. 192.168.0.110 for the pc and 192.168.0.71 for the scanner. 
![Windows ping](./ipconfig_windows.PNG "windows ip config")
##test connection under Linux
to test the settings under the Linux target system you can use netcat to check if a TCP connection to the scanner can be established
```nc -z -v -w5 $SCANNERIPADDRESS 2112```

the connection can be successfully established
![Linux netcat scanner](./nc_scanner.PNG "linux netcat scanner")

unlike a ping, the connection attempt to the hostpc is not successful 
![Linux netcat host](./nc_win_host.PNG "linux netcat host")
------------------------------------------------------------------------

![SICK Logo](https://sick-syd.data.continum.net/static_2018013123/_ui/desktop/common/images/base/pics/logo.png "SICK Logo")
![Lehning Logo](http://www.lehning.de/style/banner.jpg "LEHNING Logo")