rm output/ -rf
cd build/
rm * -rf
cmake ..
make -j8
cd ..