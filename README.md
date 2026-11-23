# `mc_reconstruction`

## Introduction

`mc_reconstruction` is part of the `mc_dev` set of repositories. The main aim of this repository is to enable cross-camera person association, tracking, and 3D reconstruction of sparse human pose detection (things like OpenPose, AlphaPose etc). The main functionality consists of:

  - Occupancy maps: Used for cross-camera person/object association
  - Occupancy tracking: Used to track objects through an occupancy map.
  - sparse pose fusion: 
  - rendering tools:
    - sparse pose detections
    - project .c3d files of markers or fused poses.
    - visually compare .c3d files of markers or fused poses.

The `mc_base` repository's README, or the CAMERA internal wiki, provide an overview of the various other parts of `mc_dev`.

## Getting the source

The source is mostly developed by Murray Evans as part of the University of Bath's CAMERA research group. The source is publicly available through CAMERA's GitHub [organisation](https://github.com/camera-mc-dev) or through the CAMERA git server, `rivendell`

`mc_reconstruction` depends on the `mc_core` and `mc_sds` repositories. If `mc_reconstruction` is pulled to `/devPath/mc_reconstruction` then it expects `/devPath/mc_core` and `/devPath/mc_sds`. 

It is recommended to use the `mc_base` repository so that all relevant things can be built in one dependency tree. As such, pull the source as:
```bash
  $ cd ~/where/you/keep/your/code
  $ git clone git@github.com:camera-mc-dev/mc_base.git mc_dev
  $ cd mc_dev
  $ git clone git@github.com:camera-mc-dev/mc_core.git
  $ git clone git@github.com:camera-mc-dev/mc_sds.git
  $ git clone git@github.com:camera-mc-dev/mc_reconstruction.git
```

or

```bash
  $ cd ~/where/you/keep/your/code
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_base mc_dev
  $ cd mc_dev
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_core
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_sds
  $ git clone camera@rivendell.cs.bath.ac.uk:mc_reconstruction
```

## Docker

The `mc_base` repository contains a number of Dockerfiles which may be appropriate to your use case, please see `mc_base` for details.

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

Obviously, you will need to install the dependencies before that will work.

## Dependencies

`mc_reconstruction` depends on everything `mc_core` and `mc_sds` depend upon. It also depends upon:

  - EZC3D: Small library for loading and saving `.c3d` files, which are a format for motion capture data.
    - This as an easy to install library that can be acquired from github and compiled and installed as per the intructions.
    - https://github.com/pyomeca/ezc3d
  - OpenSim (optional): OpenSim is a beastly creation used for doing physical simulations of bodies, particularly human bodies, and is popular among the biomechanics community. The output of `mc_reconstruction` can be used with `mc_opensim` to perform IK fits of an OpenSim body model to the 3D poses - as such, we provide a tool to visualise the opensim fit over the video data.
    - This can be a PITA installation. See `mc_opensim` for some advice.
    - This is an _optional_ dependency.


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
  - `trackSparsePoses`: This uses an OccupancyMap and an OccupancyTracker to resolve the cross-camera associations and track detected people through a scene, where the person detections come from a sparse-pose detector such as OpenPose, AlphaPose, etc.
  - `trackPoses`: More generic version of `trackSparsePoses` designed for use with sparse poses but also dense pose / segmentation sources.
  - `fuseSparsePoses`: Once you have resolved the cross-camera associations and tracked people through a scene using `trackSparsePoses`, this will perform 3D pose fusion of the individual people in each frame.
  - `timeAlign`: This special tool was used to time align marker based motion capture data with video for CAMERA's BioCV dataset (Coming Soon!)
  - rendering tools:
    - `renderSparsePose`: Draws sparse pose detections over the images.
    - `projectMocap`: Tool to project `.c3d` motion capture data into calibrated camera images. The tool can handle multiple files, but there must be only one track with any given name.
    - `compareMocap`: Tool to project multiple `.c3d` motion capture files into calibrated camera images. Can handle multiple files that have the tracks with the same names.
    - `renderOpenSim`: Tool to project an opensim model onto image data. Particularly useful in conjunction with `mc_opensim`

Full documentation of each tool, including algorithmic insights, are available.
    
## Common use cases

### 3D reconstruction of OpenPose detections.

  1) Capture calibrated and synchronised multi-camera video of your scene. For example, see the BioCV dataset. `mc_core` provides tools for calibrating a network of cameras.
  2) Run OpenPose on each camera view, producing a directory of `.json` files for each view.
  3) Run `trackSparsePoses` to resolve cross-camera person identities between camera views.
  4) Run `fuseSparsePoses` to reconstruct the sparse poses in 3D
  5) Render the output using one or more tools
  6) use `mc_opensim` to process the fused poses with OpenSim

## Documentation

You can find more documentation, such as how to configure and use the tools, structure of the library, and algorithm details, under the `docs/` folder in the repository. You can either directly read the markdown files, or 'compile' that into an html book.

To make the html book of the documentation:

  - install pandoc: `sudo apt install pandoc`
  - enter the docs directory: `cd docs/`
  - run the `.book` file: `./mc_core.book`
    - the `.book` file is actually a python script which wraps up calls to `pandoc` for making a nice html book from the markdown files.


## Building Docker image

1. pull or build mc_core:4.6.0 (see mc_core:opencv4 branch for details on how to build that)
2. run `git clone git@github.com:camera-mc-dev/mc_sds.git` Note: it needs to be a subdirectory to Dockerfile (even if you have `mc_sds`, the path needs to be `/mc_sds` relative to this directory) 
3. run `(sudo) docker build . -t mc_rec:4.6.0`
