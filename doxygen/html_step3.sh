cd ..
mkdir html/docs
cd html/docs/
rm -f *
cp ../../doxygen/docs/* .
# GENERATE BLANK Doxyfile
# doxygen -s -g Doxyfile
cat <<EOF >mainpage.dox
/**
@brief Documentation for sick_scan
@author Michael Lehning
@file Mainpage.dox
*/
/**
@mainpage sick_scan API
This contains the source code documentation for the sick_scan driver
*/
EOF
cd ..
cd ..
cd doxygen

