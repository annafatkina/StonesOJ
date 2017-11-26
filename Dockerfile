FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# denotes whether shared or static tensorflow is built
ARG shared=ON

RUN apt-get -y update
RUN apt-get -y install software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get -y update
RUN apt-get -y install build-essential curl git cmake unzip autoconf autogen libtool mlocate zlib1g-dev \
                       g++-6 python python3-numpy python3-dev python3-pip python3-wheel wget
RUN apt-get -y clean

# when building TF with Intel MKL support, `locate` database needs to exist
RUN updatedb

# install bazel for the shared library version
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN apt-get -y update \
 && apt-get -y install openjdk-8-jdk bazel \
 && apt-get -y clean

# copy the contents of this repository to the container
COPY /tensorflow_cc /tensorflow_cc
COPY /prototype /prototype
COPY /cgal-pr /cgal-pr

RUN export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# install tensorflow
RUN mkdir /tensorflow_cc/tensorflow_cc/build
WORKDIR /tensorflow_cc/tensorflow_cc/build
# configure only shared or only static library
RUN if [ "${shared}" = "OFF" ]; then \
        cmake ..; \
    else \
        cmake -DTENSORFLOW_STATIC=OFF -DTENSORFLOW_SHARED=ON ..; \
    fi
# build
RUN make
# cleanup after bazel
RUN rm -rf ~/.cache
# install
RUN make install
#cleanup
WORKDIR /
RUN rm -rf tensorflow_cc

#install OpenCV
RUN apt-get -y install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
		       libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
RUN git clone https://github.com/opencv/opencv.git
WORKDIR /opencv/
RUN mkdir release
WORKDIR /opencv/release
RUN cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_CUDA=OFF -DBUILD_opencv_gpu=OFF ..
RUN make
RUN make install
#cleanup
WORKDIR /
RUN rm -rf opencv

#install CGAL
RUN apt-get update && apt-get install -y \
    build-essential cmake \
    tar \
    libboost-dev libboost-program-options-dev \
    libboost-thread-dev libgmp10-dev \
    libmpfr-dev zlib1g-dev \
    libeigen3-dev libglew1.5-dev libipe-dev \
    libmpfi-dev libqglviewer-dev \
    libtbb-dev \
    qtbase5-dev qtscript5-dev libqt5svg5-dev qttools5-dev qttools5-dev-tools
#downloading
RUN wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.11/CGAL-4.11.tar.xz
RUN tar -xvf CGAL-4.11.tar.xz
#configuring and building
WORKDIR /CGAL-4.11/
RUN mkdir build
WORKDIR /CGAL-4.11/build/
RUN cmake ..
RUN make
RUN make install
#cleanup
WORKDIR /
RUN rm -rf CGAL-4.11.tar.xz
RUN rm -rf CGAL-4.11

