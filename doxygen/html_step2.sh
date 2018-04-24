cd ..
mkdir ./html
cd html
# Remove any documentation that is currently in this folder
cd html
rm -f *.png
rm -f *.html
rm -f *.map
rm -f *.md5
rm -f *
git clone https://github.com/michael1309/sick_scan.git .
git remote add origin/gh-pages
git checkout -b gh-pages
git branch -d master
# rm -r *
cat <<EOF >README.md
# A simple README file for the gh-pages branch of sick_scan
For further details see [here](html/index.html)
EOF
git add README.md
git commit -m"Replaced gh-pages html with simple readme"
cd ..
cd ../doxygen




