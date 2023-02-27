## Rendering Sparse Pose, fusion, and .c3d data

It is, obviously, very valuable to render your raw pose detections, fusion results, and ground truth
mocap data, back onto the images/video you are working with.

As such, there are a number of tools you can use:

  - `renderSparsePose`: Renders raw pose detections onto video source
  - `projectMocap`:     Renders .c3d files from fusion or a mocap system onto video source as markers
  - `compareMocap`:     Renders multiple .c3d files from fusion or mocap as a skeleton


### Rendering raw detections

A simple tool to render sparse pose detections is `renderSparsePose` which can be run as:

   `renderSparsePose <image source> <skeleton config> <pose data type> <pose source>`

The tool will render circles at each keypoint, adjusting the brightness to correspond to the confidence.
It will also draw a line from each keypoint to its parent keypoint (as defined in the skeleton config) - 
note that a keypoint with itself as its parent will just be a point - useful if there are some lines you
don't want to draw. The pose data type and pose source must be as per the fusion config - a directory of
.json files as per OpenPose or DeepLabCut .csv files. 

### Rendering mocap markers from .c3d files.

The `projectMocap` tool can be used to render markers from `.c3d` files - whether those files come from
a motion capture system or from the output of pose fusion.

NOTE: `.trc` files from OpenSim can also now be used but is "experimental" / debug / a hack!

The tool takes input from a config file:

  `./projectMocap <config>`

If needs be, the tool can render "headless" - writing output to a video file instead of displaying on screen.

#### projectMocap config

```bash
#
# Start with the usual path information.
#
dataRoot = "/path/to/datasets/"
testRoot = "path/to/trial/"

# then specify the video source
# multiple sources can be provided, and swapped between using the arrow keys.
vidFiles = (
              "00.mp4"
           );

# and the calibration for the video source(s)
calibFiles = (
               "00.mp4.calib"
             );

# (optional) - if there is a time offset between the .c3d data and video data
# you can specify the offset in an offset file
offsetFile = "frameOffset";

# specify the .c3d files. You can use multiple files, but if the marker names 
# clash between files you will get an error.
trackFiles = ("track0001.c3d");

# should we visualise? You will want to say true - the option to not visualise
# is a legacy provision
visualise = true;

# (optional) specify a headless render, and where to write the render to.
# When doing a headless render, only specify a single input video source.
#renderHeadless = true;
#renderTarget = "proj-00.mp4";
```


### Comparing .c3d files with skeleton renders

You may want to visualise the difference between 2 or more .c3d files, in which case you can use the `compareMocap` tool.
As with `projectMocap` the tool required a config file detailing data sources etc

   `./compareMocap <config>`

#### compareMocap config

```bash
#
# Start with the usual path information.
#
dataRoot = "/path/to/datasets/"
testRoot = "path/to/trial/"

# then specify the video source
# multiple sources can be provided, and swapped between using the arrow keys.
vidFiles = (
              "00.mp4"
           );

# and the calibration for the video source(s)
calibFiles = (
               "00.mp4.calib"
             );

# (optional) - if there is a time offset between the .c3d data and video data
# you can specify the offset in an offset file
offsetFile = "frameOffset";

# specify the marker files, and also for each marker file, specify the skeleton configuration
trackFiles    = ("op-recon/body-00.c3d", "op-recon/body-00-smoothed.c3d");
skeletonFiles = ("../open.skel.cfg", "../open.skel.cfg");

# should we visualise? You will want to say true - the option to not visualise
# is a legacy provision
visualise = true;

# (optional) specify a headless render, and where to write the render to.
# When doing a headless render, only specify a single input video source.
#renderHeadless = true;
#renderTarget = "proj-00.mp4";
```


