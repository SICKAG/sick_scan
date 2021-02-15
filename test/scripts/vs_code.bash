#!/bin/bash

gedit ./make.bash ./run_simu_tim7xx_tim7xxS.bash & 

pushd ../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash

code ./sick_scan_vscode.code-workspace
popd 
