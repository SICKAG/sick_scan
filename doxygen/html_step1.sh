#
# Prepare html-Subdirectory for doxygen
#
cd ..
mkdir ./html
echo "html/" >> .gitignore
git add html .gitignore
git commit -m "Added html folder (which will contain the gh-pages branch) and .gitignore list"
git push origin master

