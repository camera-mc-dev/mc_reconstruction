FROM --platform=linux/amd64 mc_core:4.6.0

# scons stuff
RUN apt update && apt install -y --no-install-recommends scons libswscale-dev libavformat-dev libconfig++-dev && rm -rf /var/lib/apt/lists/* 

WORKDIR /home/mc_dev

# mc_sds

WORKDIR /home/mc_dev/mc_sds/
COPY mc_sds .
RUN scons -j8

# mc_rec

# specific dependencies
WORKDIR /deps/
RUN git clone https://github.com/pyomeca/ezc3d.git
WORKDIR /deps/ezc3d/build
RUN cmake -DBUILD_EXAMPLES=FALSE ..
RUN make -j8
RUN make install

WORKDIR /home/mc_dev/mc_reconstruction/
COPY . .
# RUN scons -j8
