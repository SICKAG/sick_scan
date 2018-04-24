cd ..
cd ./html
doxygen ./docs/Doxyfile
cd html
git add .
git commit -m "Added Doxygen output to repo"
git push origin gh-pages
cd ../../doxygen

