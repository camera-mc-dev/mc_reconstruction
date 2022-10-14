FROM --platform=linux/amd64 mc_core:4.6.0

# scons stuff
RUN apt update && apt install -y --no-install-recommends scons libswscale-dev libavformat-dev libconfig++-dev && rm -rf /var/lib/apt/lists/* 

WORKDIR /home/mc_dev

# -----------------
# mc_sds
# -----------------

WORKDIR /home/mc_dev/mc_sds/
COPY mc_sds .
RUN scons -j8

WORKDIR /deps/
# ezc3d
RUN git clone https://github.com/pyomeca/ezc3d.git
WORKDIR /deps/ezc3d/build
RUN cmake -DBUILD_EXAMPLES=FALSE ..
RUN make -j8
RUN make install

# --------------------
# opensim
# --------------------
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

# ----------------
# mc_rec
# ----------------
WORKDIR /home/mc_dev/mc_reconstruction/
COPY . .
RUN scons -j8

WORKDIR /home
