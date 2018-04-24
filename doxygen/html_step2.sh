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
git remote add origin/gh-pages
git checkout origin/gh-pages -b gh-pages
git branch -d master
# rm -r *
cat <<EOF >README.md
# A simple README file for the gh-pages branch of sick_scan
For further details see [here](html/index.html)
EOF
git add README.md
git commit -m"Replaced gh-pages html with simple readme"
cd ..
cd doxygen




