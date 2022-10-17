FROM nvidia/cuda:11.3.0-devel-ubuntu20.04
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# system installs
# -----------------------------

RUN apt-get update
RUN apt-get -y install build-essential
RUN ln -s /usr/share/zoneinfo/Europe/London /etc/localtime
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata
RUN apt-get -y install git
RUN apt-get -y install libcudnn8-dev
RUN apt-get -y install wget
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install intel-mkl-full


#-------------------------------
# add git key for cloning
#-------------------------------

RUN mkdir /root/.ssh/
ADD id_rsa /root/.ssh/id_rsa

# Create known_hosts
RUN touch /root/.ssh/known_hosts

RUN ssh-keyscan github.com >> /root/.ssh/known_hosts



#-----------------------------
# dependencies for mc_dev and opencv
#-----------------------------

RUN apt -y update
RUN apt -y upgrade
RUN apt install -y \
	--no-install-recommends \
	libsfml-dev \
	libglew-dev \
	libfreetype-dev \
	libegl-dev \
	libeigen3-dev \
	libboost-filesystem-dev \
	libmagick++-dev \
	libconfig++-dev \
	libsnappy-dev \
	libceres-dev \
	ffmpeg \
        libswscale-dev \
        libavformat-dev \
        scons \
        cmake
#---------------------------
# opencv
#---------------------------

WORKDIR /deps
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
WORKDIR /deps/opencv_contrib
RUN git checkout 4.6.0
WORKDIR /deps/opencv
RUN git checkout 4.6.0
RUN mkdir build

WORKDIR build/
RUN cmake -DCMAKE_BUILD_TYPE=RELEASE \
   -DINSTALL_C_EXAMPLES=OFF \
   -DINSTALL_PYTHON_EXAMPLES=OFF \
   -DOPENCV_ENABLE_MEMORY_SANITIZER=ON \
   -DOPENCV_ENABLE_NONFREE=ON \
   -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
   -DENABLE_PRECOMPILED_HEADERS=OFF \
   -DOPENCV_GENERATE_PKGCONFIG=ON \
   -DBUILD_EXAMPLES=OFF \
   -DWITH_CUDA=ON \
   -DWITH_OPENMP=ON .. 

RUN make -j8
RUN make install


WORKDIR /home/mc_dev

# and we need HighFive for hdf5 files
WORKDIR /deps/
RUN apt update
RUN apt install libhdf5-dev -y --no-install-recommends
RUN apt install libboost-serialization-dev -y --no-install-recommends
RUN git clone https://github.com/BlueBrain/HighFive.git
WORKDIR /deps/HighFive
RUN mkdir build
WORKDIR /deps/HighFive/build
RUN cmake HIGHFIVE_UNIT_TESTS=OFF ../
RUN make -j6
RUN make install

# and we use nanoflann in various places
WORKDIR /deps/
RUN git clone https://github.com/jlblancoc/nanoflann.git
WORKDIR /deps/nanoflann
RUN mkdir build
WORKDIR /deps/nanoflann/build
RUN cmake ../
RUN make -j6
RUN make install

WORKDIR /home/mc_dev/
RUN git clone git@github.com:camera-mc-dev/mc_core.git
WORKDIR /home/mc_dev/mc_core/
RUN git checkout opencv4
RUN scons -j8

# -------------------------
# mc_sds
# -------------------------

WORKDIR /home/mc_dev/
RUN git clone git@github.com:camera-mc-dev/mc_sds.git
WORKDIR /home/mc_dev/mc_sds/
RUN git fetch
RUN git checkout opencv4
RUN scons -j8

WORKDIR /deps/
# ezc3d
RUN git clone https://github.com/pyomeca/ezc3d.git
WORKDIR /deps/ezc3d/build
RUN cmake -DBUILD_EXAMPLES=FALSE ..
RUN make -j8
RUN make install

# -------------------------
# opensim
# ------------------------

WORKDIR /deps/
# deps for osim
RUN apt update && apt install -y libsimbody-dev libspdlog-dev freeglut3-dev libxi-dev libxmu-dev \
                           liblapack-dev swig3.0
# docopt
RUN git clone https://github.com/docopt/docopt.cpp.git
WORKDIR /deps/docopt.cpp/build/ 
RUN cmake ..
RUN make -j8
RUN make install

WORKDIR /deps/
RUN git clone https://github.com/opensim-org/opensim-core.git

WORKDIR /deps/opensim_dependencies_build
RUN cmake ../opensim-core/dependencies/ \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
RUN make -j8

WORKDIR /deps/opensim_build
RUN cmake ../opensim-core \
      -DBUILD_API_EXAMPLES=OFF\
      -DBUILD_TESTING=OFF\
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DOPENSIM_DEPENDENCIES_DIR="/deps/opensim_dependencies_install" \
      -DOPENSIM_WITH_CASADI=OFF \
      -DOPENSIM_WITH_TROPTER=OFF 
RUN make -j8
RUN make install

# ---------------------
# mc_rec
# ---------------------

WORKDIR /home/mc_dev/mc_reconstruction/
COPY . .
RUN scons -j8

WORKDIR /home

