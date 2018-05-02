^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_scan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* moved some cpp files to ensure Debian compatibility
* Contributors: Unknown

0.0.12 (2018-04-25)
-------------------
* Added script to start all test sequentially
* Added RSSi and Range Deviation Test to sick_scan_test
* channel handling for 8 bit rssi values corrected
* Defines for param keyword introduced
* added ros param for rssi data size 16 or 8 Bit
* added rssi resolution configswitch
* support for LMS_5xx and LMS_1xx added
* testprogramm can now handle comments;
* Test instructions added
* Generation of result file
* inital test revisited
* Initial version protocol tester
* Tiny XML Parser added
* added Sopas protocol param
* Added Tools and driver folder, removed unnecessary libusb dep.
* Added scanner_type to parameter set to allow the processing of parallel scanners
* timeout handling improved
* reading thread times after connection lost
  Timeout settings optimized
* protocol switching supported
* Protocol switching implemented
* added timeout and binary/ascii detection
* Support of LMS1104 debugged, skipping scan mgs. publish for MRS6124 (only pointcloud)
* Adding MRS6124 link to supported scanner table
  Edited trouble shooting
* Add documentation for network stack
* scandataCfg for binary commands prepared
* min_ang, max_ang adapted for MRS6xxx
* LMS1000 support continue, Bug fix for parsing distance value MRS6xxx, mrs6xxx.launch modified
* COLA_A and COLA_B prepared
* Package handling optimized (for asynchron tcp data transfer)
* Debug info added for receiving tcp packets
* Support of MRS1104
* Cleanup and supporting Tim571
* errorhandler added
* First version with 9413 bytes packet
* tcp handling optimized
* Queue introduced
* colaa+colab libs included
* Parsing of MRS6xxx-data packages integrated
* Timeout incremented due to startup wait phase for MRS6xxx
* Sleep duration between inital commands changed from 2.0 to 0.2
  Sleep of 10 Sec. introducted after start scandata to ensure that the scanner comes up.
0.0.11 (2018-01-24)
-------------------
