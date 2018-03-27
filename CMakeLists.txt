cmake_minimum_required(VERSION 2.8.3)
project(sick_scan)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure)

find_package(PkgConfig REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(cfg/SickScan.cfg)

catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure
    LIBRARIES sick_scan_lib
    INCLUDE_DIRS include )

include_directories(include include/tcp ${catkin_INCLUDE_DIRS} )

add_library(sick_scan_lib
    driver/src/sick_scan_common.cpp
    driver/src/abstract_parser.cpp
	driver/src/tcp/tcp.cpp
	driver/src/tcp/Mutex.cpp
	driver/src/tcp/SickThread.cpp
    driver/src/tcp/errorhandler.cpp
    driver/src/tcp/toolbox.cpp
    driver/src/tcp/Time.cpp
    driver/src/tcp/colaa.cpp
    driver/src/tcp/colab.cpp
	driver/src/binScanf.cpp	)
add_dependencies(sick_scan_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sick_scan_lib
    ${catkin_LIBRARIES})

add_executable(sick_generic_caller
    driver/src/sick_generic_laser.cpp
    driver/src/sick_generic_caller.cpp
    driver/src/sick_scan_common_tcp.cpp
    driver/src/sick_generic_parser.cpp
	driver/src/sick_scan_common_nw.cpp
	)
target_link_libraries(sick_generic_caller sick_scan_lib)


install(TARGETS sick_scan_lib
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
        sick_generic_caller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/${PROJECT_NAME}/abstract_parser.h
              include/${PROJECT_NAME}/sick_scan_common.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
