#
#
# Doxygen preparation for github.io
#
cd ..
mkdir ./html
cd html
lastpart=$(echo $(basename $PWD))
if [ "$lastpart" = "html" ]
then
  echo "html subdir found - starting cleanup process"
else
  echo "The script does not run in an html subdir - aborting"
  exit 0
fi
# Remove any documentation that is currently in this folder
rm -rf *
rm -rf .git
rm -rf .gitignore
rm -rf .*

git clone https://github.com/michael1309/sick_scan.git .
cd ..
mkdir html/docs
cd html/docs/
rm -rf *
cp ../../doxygen/docs/* .
cd ..
doxygen ./docs/Doxyfile
git checkout gh-pages
cd ./html
git add .
cd ..
git commit -m "Added Doxygen output to repo"
git push origin gh-pages
cd ../doxygen

