Steps to run:

1) Install Tensorflow with GPU support to *${PROJECT_SOURCE_DIR}/install/tensorflow*

In example use this manual: https://github.com/FloopCZ/tensorflow_cc

2) Install OpenCV to *${PROJECT_SOURCE_DIR}/install/opencv*

3) Download [Tensorflow model](http://campar.in.tum.de/files/rupprecht/depthpred/NYU_FCRN-checkpoint.zip), unzip it and copy to *${PROJECT_SOURCE_DIR}/build/*

4) 
```
cd build
cmake ..
make
./prototype -i <path/to/input/video/file> -o <path/to/output/video/file> -r
```
