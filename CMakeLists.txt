####################################################
# CMakeLists.txt for project sick_scan
#
#
####################################################
cmake_minimum_required(VERSION 2.8.3)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(sick_scan)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        endif ()
    else ()
        set(CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

# Switch on, if you use c11-specific commands
use_cxx11()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-overflow")

find_package(Boost REQUIRED COMPONENTS system serialization)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  geometry_msgs
  message_generation
  roscpp
  roslib # needed ros::package::getPath()
  sensor_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
  )


find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/SickScan.cfg
        cfg/tf_dyn.cfg
)

# driver messages
add_message_files(
        DIRECTORY msg
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
        LFErecFieldMsg.msg
        LFErecMsg.msg
        LIDoutputstateMsg.msg
)

# driver services
add_service_files(
        DIRECTORY srv
        FILES
        ColaMsgSrv.srv
        ECRChangeArrSrv.srv
        LIDoutputstateSrv.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp sensor_msgs diagnostic_updater tf tf2
        LIBRARIES sick_scan_lib
        INCLUDE_DIRS include
        DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS}  include/sick_scan)

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
        driver/src/helper/angle_compensator.cpp
        driver/src/sick_generic_field_mon.cpp
        driver/src/sick_scan_marker.cpp
        driver/src/sick_scan_messages.cpp
        driver/src/sick_scan_services.cpp
        )

add_dependencies(sick_scan_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(sick_scan_lib
        ${catkin_LIBRARIES})

add_executable(sick_generic_caller
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_caller.cpp
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
