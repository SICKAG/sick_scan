####################################################
# CMakeLists.txt for project sick_scan
#
#
####################################################
cmake_minimum_required(VERSION 2.8.3)


macro(use_cxx11)
	if (CMAKE_VERSION VERSION_LESS "3.1")
		if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
			set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
		endif ()
	else ()
		set (CMAKE_CXX_STANDARD 11)
	endif ()
endmacro(use_cxx11)

#set(CMAKE_BUILD_TYPE Debug) #uncomment to activate debug mode for lib_sick as well
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++03")

# Switch on, if you use c11-specific commands
use_cxx11()

# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")

project(sick_scan)

# By uncommenting the following lines you can list all variable names ...
# --- CUT ---
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#	message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# --- CUT ---

# For warning <catkin_workspace>/devel/include' should be placed in
# the devel space instead of the build space
# see: https://answers.ros.org/question/67244/qtcreator-with-catkin/



# message(${CMAKE_BINARY_DIR})

# set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/devel")
# set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/devel;/opt/ros/melodic")


find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure
      geometry_msgs
      std_msgs
      sensor_msgs
      visualization_msgs
      message_generation
      pcl_conversions
      pcl_ros
)


find_package(PkgConfig REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
		cfg/SickScan.cfg
		cfg/tf_dyn.cfg
)

add_message_files(

		FILES
		SickImu.msg
		RadarObject.msg
		RadarPreHeaderDeviceBlock.msg
		RadarPreHeaderMeasurementParam1Block.msg
		RadarPreHeaderStatusBlock.msg
		RadarPreHeaderEncoderBlock.msg
		RadarPreHeader.msg
		RadarScan.msg
		Encoder.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        DEPENDS Boost
        CATKIN_DEPENDS message_runtime roscpp sensor_msgs diagnostic_updater dynamic_reconfigure
        LIBRARIES sick_scan_lib
        INCLUDE_DIRS include )

include_directories(include include/tinyxml ${catkin_INCLUDE_DIRS} include/sick_scan)

add_library(sick_scan_lib
		driver/src/dataDumper.cpp
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
	    driver/src/binScanf.cpp
        driver/src/sick_scan_common_tcp.cpp
	    driver/src/sick_generic_radar.cpp
	    driver/src/sick_generic_imu.cpp
	    driver/src/sick_generic_parser.cpp
	    driver/src/sick_scan_common_nw.cpp
		include/sick_scan/sick_scan_config_internal.h
		driver/src/sick_scan_config_internal.cpp
		include/sick_scan/softwarePLL.h
        driver/src/softwarePLL.cpp
		)

add_dependencies(sick_scan_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(sick_scan_lib
    ${catkin_LIBRARIES})

add_executable(sick_generic_caller
    driver/src/sick_generic_laser.cpp
    driver/src/sick_generic_caller.cpp
        )

#
#  radar_object_marker (receives radar msg. and publishes marker array for rviz or similar
#
add_executable(radar_object_marker
        tools/radar_object_marker/src/radar_object_marker.cpp
		tools/pcl_converter/src/gnuplotPaletteReader.cpp
        include/radar_object_marker/radar_object_marker.h)
target_link_libraries(radar_object_marker sick_scan_lib)
#
#
#



add_executable(pcl_converter tools/pcl_converter/src/pcl_converter.cpp tools/pcl_converter/src/gnuplotPaletteReader.cpp)

target_link_libraries(pcl_converter
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES})


# add_executable( pcl_find_plane tools/pcl_converter/src/pcl_find_plane.cpp)
# target_link_libraries(pcl_find_plane ${catkin_LIBRARIES})


target_link_libraries(sick_generic_caller sick_scan_lib)

add_executable(sick_scan_test
	test/src/sick_scan_test.cpp
	driver/src/tinyxml/tinystr.cpp
	driver/src/tinyxml/tinyxml.cpp
	driver/src/tinyxml/tinyxmlerror.cpp
	driver/src/tinyxml/tinyxmlparser.cpp
	)
target_link_libraries(sick_scan_test
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        sick_scan_lib)

#
#  Sensor alignment tool
#
#  currently disabled for trusty support
# add_executable(sensor_alignment
#		tools/sensor_alignment/sensor_alignment_tf_dyn.cpp
#		)
#
# target_link_libraries(sensor_alignment
#		${catkin_LIBRARIES}
#		)
# add_dependencies(sensor_alignment ${PROJECT_NAME}_gencfg)

# target_link_libraries(${PROJECT_NAME}_pointcloud2_refresh
#		${catkin_LIBRARIES}
#	)


# Destinations:
# LIB for library
# BIN for Binaries
# INCLUDE for Includes
# SHARE for config files, launch files etc.


install(TARGETS sick_scan_lib
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
        sick_generic_caller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
		TARGETS
		sick_scan_test
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(FILES include/${PROJECT_NAME}/abstract_parser.h
              include/${PROJECT_NAME}/sick_scan_common.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY test/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test 
	FILES_MATCHING PATTERN "*.xml"
	PATTERN "test/src" EXCLUDE)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
