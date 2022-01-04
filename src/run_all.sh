cd build
cmake ..
make
./plt /mnt/host/dataset # result csv will be placed on build/FpsVsPointnum.csv
cd ../
python3.8 visualizer.py # Result png will be placed on ./FpsVsPointnum.png