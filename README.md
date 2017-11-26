Steps to run:

1) Install [docker](https://docs.docker.com/engine/installation/) and [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)

2) 
```
sudo docker build -t prototype:gpu .
sudo nvidia-docker run -it prototype:gpu

cd prototype
mkdir build
cmake ..
make
```

3) Download [Tensorflow model](http://campar.in.tum.de/files/rupprecht/depthpred/NYU_FCRN-checkpoint.zip), unzip it and copy to /prototype/build/

4) 
```
./prototype -i <path/to/input/video/file> -o <path/to/output/video/file> -r
```
