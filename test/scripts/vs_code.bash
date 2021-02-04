#!/bin/bash

gedit ./make.bash ./run_simu_tim781s.bash & 

pushd ../../../..
source /opt/ros/melodic/setup.bash
source ./install/setup.bash

code ./sick_scan_vscode.code-workspace
popd 
