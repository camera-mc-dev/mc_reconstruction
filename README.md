# `mc_reconstruction`

## Introduction

`mc_reconstruction` is part of the `mc_dev` set of repositories and contains functionality for 3D reconstruction. The main fuctionality is

  - Occupancy maps
  - Occupancy tracking
  - sparse pose fusion
  - rendering of fused poses.

The CAMERA wiki provides an overview of the various other parts of `mc_dev`.

## Getting the source

The source is mostly developed by Murray Evans as part of the university of Bath's CAMERA research group. The source can be pulled from the CAMERA git server *rivendell* - please see the CAMERA wiki for details on using and accessing the git server.

`mc_reconstruction` depends on the `mc_core` and `mc_sds` repositories. If `mc_reconstruction` is pulled to `/devPath/mc_reconstruction` then it expects `/devPath/mc_core` and `/devPath/mc_sds`. 

It is recommended to use the `mc_base` repository as well so that all relevant things can be built in one dependency tree. As such, pull the source as:

```bash
  $ cd ~/where/you/keep/your/code
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_base mc_dev
  $ cd mc_dev
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_core
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_sds
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_reconstruction
```

## Building

The project is built using the `scons` build system. If you have not already done so, you will need to install `scons` now, probably using your package manager or Homebrew on Mac. e.g. for Ubuntu based systems:

```bash
  $ sudo apt install scons
```

Next, change into your `mc_base` directory, e.g.:

```bash
  $ cd ~/where/you/keep/your/code/mc_dev/
```

To build everything in optimised mode just type (using 5 build jobs):

```bash
  $ scons -j5
```

Obviously, that will fail if you have not yet installed all the required dependencies or modified the build to find those dependencies.

## Dependencies

`mc_reconstruction` depends on everything `mc_core` and `mc_sds` depend upon. It also depends upon:

  - EZC3D: Small library for loading and saving `.c3d` files, which are a format for motion capture data.
    - This as an easy to install library that can be acquired from github and compiled and installed as per the intructions.
    - https://github.com/pyomeca/ezc3d


## Specifing where dependencies are

As with any software build, you will need to tell the build system where to find the libraries that it needs. This is currently done by the `mc_core/mcdev_recon_config.py` script.

For each dependency you will find a `Find<X>()` function in the script. For most of the dependencies this just wraps calls to the way `scons` uses `pkg-config` type tools. Unless you've done something non-standard with your installs, then these defaults will probably satisfy you. Scons handle this with:

```python
env.ParseConfig("pkg-config libavformat --cflags --libs")
```

Basically, you pass in the same string you would use if you typed it on the command line.

We may in time modify these `Find<X>()` functions to do the hunting for you like `CMake` promises (and normally fails) to do. Anyway.

Scons is pretty simple.

Need to specify an include path?

```python
env.Append(CPPPATH=['/path/to/include/'])
```

Need to add a library path?

```python
env.Append(LIBPATH=['/path/to/lib/'])
```

Need to add a library?

```python
env.Append(LIBS=['grumpy'])  # adds libgrumpy.a or .so or whatever.
```

Need to specify a pre-processor define or other C++ flag?

```python
env.Append(CPPFLAGS=['-Wall', '-DUSE_PIZZA' ])
```

Need to specify linker flags?

```python
env.Append(LINKFLAGS=['-framework', 'OpenGL'])
```

For more details on how the build system is set up, see the main documentation.



## Tools

`mc_reconstruction` supplies the following tools:

  - `occTrack`: Given a set of calibrated image sources, where each source supplies binary mask images, this is a demonstration tool for how to make use of the occupancy map and occupancy tracker classes. The tool will perform basic tracking of the masked out objects in the scene on a ground plane.
  - `trackSparsePoses`: This uses and OccupancyMap and an OccupancyTracker to resolve the cross-camera associations and track detected people through a scene, where the person detections come from a sparse-pose detector such as OpenPose, AlphaPose, etc.
  - `fuseSparsePoses`: Once you have resolved the cross-camera associations and tracked people through a scene using `trackSparsePoses`, this will perform 3D pose fusion of the individual people in each frame.
  - `projectMocap`: Tool to project `.c3d` motion capture data into calibrated camera images. The tool can handle multiple files, but there must be only one track with any given name.
  - `compareMocap`: Tool to project multiple `.c3d` motion capture files into calibrated camera images. Can handle multiple files that have the tracks with the same names.

## Documentation

You can find more documentation, such as how to configure and use the tools, structure of the library, and algorithm details, under the `docs/` folder in the repository. You can either directly read the markdown files, or 'compile' that into an html book.

To make the html book of the documentation:

  - install pandoc: `sudo apt install pandoc`
  - enter the docs directory: `cd docs/`
  - run the `.book` file: `./mc_core.book`
    - the `.book` file is actually a python script which wraps up calls to `pandoc` for making a nice html book from the markdown files.


## Building Docker image
1. make sure nvidia-container-runtime is installed on your machine. Installation instructions (for linux) can be found here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
2. run `git clone git@github.com:camera-mc-dev/mc_reconstruction.git` 
3. copy a valid ssh-key to the repository to `/mc_reconstruction/` root directory (the same directory as the Dockerfile) and name it id_rsa. This will be copied into the docker build container and deleted when it completes.
4. run `(sudo) docker build . -t mc_rec:4.6.0`
