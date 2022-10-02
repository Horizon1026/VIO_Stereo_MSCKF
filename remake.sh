sh update_lib.sh
cd build/
rm ./* -rf
cmake ..
make -j8
cd ..