#
# Stops the communication app started by roslaunch.
# After stopping you can start your debug ide to start sick_generic_caller in the ros environment. 
#
# use use pkill -f sick_generic_caller
#
echo "Killing all sick_generical_caller instances"
for KILLPID in `ps ax | grep 'sick_generic_caller' | grep -v 'grep' | awk ' { print $1;}'`; do
  kill -9 $KILLPID;
done
