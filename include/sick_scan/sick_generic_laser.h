#ifndef SICK_GENERIC_LASER_H
#define SICK_GENERIC_LASER_H
/*
 *
 */
#ifndef _MSC_VER
#include <sick_scan/sick_scan_common_usb.h>
#endif
#include <sick_scan/sick_scan_common_tcp.h>
#include <sick_scan/sick_scan_common_mockup.h>

int mainGenericLaser(int argc, char **argv, std::string scannerName);
#endif

