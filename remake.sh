sh update_lib.sh
mkdir build
cd build/
rm ./* -rf
cmake ..
make -j8
cd ..