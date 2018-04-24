#!/bin/sh 
for i in *.xml; do 
    [ -f "$i" ] || break 
    basename=$(basename $i) 
    filename='test/'$basename 
    ipaddress=$( grep -E -o "([0-9]{1,3}[\.]){3}[0-9]{1,3}" $i)
    ping -c 1 $ipaddress
    if [ $? -eq 0 ] 
    then 
      echo 'scanner is alive Starting Test'
      yes 2| rosrun sick_scan sick_scan_test $filename 
    else 
    echo 'scanner is not reachable skipping test'
    echo $filename
    echo 'Skipped'
    fi
    echo 'sleeping 5 seconds before starting next Test'
    sleep 5
done 
 
 
