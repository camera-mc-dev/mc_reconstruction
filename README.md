# `mc_reconstruction`

## Introduction

`mc_reconstruction` is part of the [mc_dev](https://www.github.com/camera-mc-dev) set of repositories. The main aim of this repository is to enable cross-camera person association, tracking, and 3D reconstruction of sparse human pose detection (things like OpenPose, AlphaPose etc). The main functionality consists of:

  - Occupancy maps: Used for cross-camera person/object association
  - Occupancy tracking: Used to track objects through an occupancy map.
  - sparse pose fusion: 
  - rendering tools:
    - sparse pose detections
    - project .c3d files of markers or fused poses.
    - visually compare .c3d files of markers or fused poses.

The `mc_base` repository's README, or the CAMERA internal wiki, provide an overview of the various other parts of `mc_dev`.

## Installation

### Easy mode - docker etc.

`mc_reconstruction` is normally used as part of the markerless motion capture pipeline developed in parallel with the BioCV dataset. If this is your intended use case, you may benefit from the [use-case specific instructions, docker containers and build helpers provided](https://github.com/camera-mc-dev/.github/blob/main/profile/mocapPipe.md) 

### Manual mode

#### Getting the source

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

#### Building

The `mc_base` and `mc_core` repositories contain instructions relevant to all `mc_dev` repositories. Refer to those for basic project layout and the SCons based build system. Extra dependencies specific to `mc_reconstructon` are explained in the following section.

Put simply, once all dependencies are installed, and assuming you used the `mc_base` repository, you will just do:

```bash
cd /path/to/mc_dev
scons -j6
```

Or to install to `/opt/mc_bin`:

```bash
cd /path/to/mc_dev
scons -j6 install=True installDir=/opt/mc_bin
```


#### Dependencies

`mc_reconstruction` depends on everything `mc_core` and `mc_sds` depend upon. It also depends upon:

  - [EZC3D](https://github.com/pyomeca/ezc3d): Small library for loading and saving `.c3d` files, which are a format for motion capture data.
    - This as an easy to install library that can be acquired from github and compiled and installed as per the intructions.
  - OpenSim (optional): OpenSim is a beastly creation used for doing physical simulations of bodies, particularly human bodies, and is popular among the biomechanics community. The output of `mc_reconstruction` can be used with `mc_opensim` to perform IK fits of an OpenSim body model to the 3D poses - as such, we provide a tool to visualise the opensim fit over the video data.
    - This can be a PITA installation.
    - This is an _optional_ dependency
    - Relatively recent versions of OpenSim introduced a new build script which reduces the pain. We cloned and modified that script and put it under `mc_reconstruction/scripts`. The main advantage of our version is the ability to control where OpenSim gets built and installed.


#### Configuration

As per `mc_core` this is handled by a small config script, in this case `mc_core/mcdev_recon_config.py`, which will be picked up when you run SCons. The format should be pretty obvious but refer to the documentation in `mc_core` for some hints.

The script will look for EZ3CD in `/usr/local` which is the default install location. 

For OpenSim, you will either need to modify the specified paths from their default `/opt/opensim/install` or comment out the whole section and replace it with `pass` like so to disable use of the library:

```python
def FindOpensim(env):
    #env.Append(CPPDEFINES=["USE_OPENSIM"]);
    #env.Append(CPPPATH=["/opt/opensim/install/sdk/include/",
    #                    "/opt/opensim/install/sdk/include/OpenSim/",
    #                    "/opt/opensim/install/sdk/spdlog/include/",
    #                    "/opt/opensim/install/sdk/Simbody/include/simbody" ])
    #env.Append(LIBPATH=["/opt/opensim/install/sdk/lib/",
    #                    "/opt/opensim/install/sdk/Simbody/lib/"])
    #env.Append(LIBS=["fmt", "osimAnalyses", "osimActuators", "osimSimulation", "osimTools", "osimCommon", "SimTKsimbody", "SimTKcommon"])
    pass
```


## Tools

`mc_reconstruction` supplies the following tools:

  - `occTrack`: Given a set of calibrated image sources, where each source supplies binary mask images, this is a demonstration tool for how to make use of the occupancy map and occupancy tracker classes. The tool will perform basic tracking of the masked out objects in the scene on a ground plane.
  - `trackSparsePoses`: Given Sparse keypoint human (or object) detections from the likes of OpenPose, this uses an OccupancyMap and an OccupancyTracker to resolve the cross-camera person/object associations and track detected people through a scene
  - `trackPoses`: More generic version of `trackSparsePoses` designed for use with sparse poses but also dense pose / segmentation sources.
  - `fuseSparsePoses`: Once you have resolved the cross-camera associations and tracked people through a scene using `trackSparsePoses`, this will perform 3D pose fusion of the individual people in each frame.
  - `timeAlign`: This special tool was used to time align marker based motion capture data with video for CAMERA's BioCV dataset (Coming Soon!) based on a flashing LED. (frames were already synched by hardware but alignment was not quite resolved by that hardware)
  - rendering tools:
    - `renderSparsePose`: Draws sparse pose detections over the images.
    - `projectMocap`: Tool to project `.c3d` motion capture data into calibrated camera images. The tool can handle multiple files, but there must be only one track with any given name.
    - `compareMocap`: Tool to project multiple `.c3d` motion capture files into calibrated camera images. Can handle multiple files that have the tracks with the same names, will colour each individual differently
    - `renderOpenSim`: Tool to project an opensim model onto image data. Particularly useful in conjunction with `mc_opensim`

Full documentation of each tool, including algorithmic insights, are available.
    
## Common use cases

### 3D reconstruction of OpenPose detections.

First, capture calibrated and synchronised multi-camera video of your scene. For example, see the BioCV dataset. `mc_core` provides tools for calibrating a network of cameras.

Next, run OpenPose (or other sparse keypoint pose detector) over the individual videos.

<div style="text-align: center">
![OpenPose on BioCV P28 data](imgs/opdet-bcv28.mp4){style="width: 90%; margin: auto;"}
</div>

Now, run the occupancy map based `trackSparsePoses` to resolve cross-camera person identities between camera views.

<div style="text-align: center">
![Occupancy map tracking (raw occupancy on left, resulting track on right)](imgs/occTrk-bcv28.mp4){style="width: 90%; margin: auto;"}
</div>

Get the 3D reconstruction of the poses using `fuseSparsePoses`, and render the output as needed (e.g. using `compareMocap` as here)

<div style="text-align: center">
![Fused OpenPose on BioCV P28 data](imgs/recon-bcv28.mp4){style="width: 90%; margin: auto;"}
</div>

Use the `mc_opensim` tools to process the fused poses with OpenSim, and if wanted, render the OpenSim model back over the images.

<div style="text-align: center">
![OpenSim render on BioCV P28 data](imgs/osim-bcv28.mp4){style="width: 90%; margin: auto;"}
</div>

## Documentation

You can find more documentation, such as how to configure and use the tools, structure of the library, and algorithm details, under the `docs/` folder in the repository. You can either directly read the markdown files, or 'compile' that into an html book.

To make the html book of the documentation:

  - install pandoc: `sudo apt install pandoc`
  - enter the docs directory: `cd docs/`
  - run the `.book` file: `./mc_core.book`
    - the `.book` file is actually a python script which wraps up calls to `pandoc` for making a nice html book from the markdown files.


