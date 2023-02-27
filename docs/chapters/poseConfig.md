## Configuration file for trackSparsePoses and fuseSparsePoses

You can use the same configuration file for both `trackSparsePoses` and `fuseSparsePoses`, as many of the configuration parameters are shared between the two.

```bash

#
# Start with the usual path information.
#
dataRoot = "/path/to/datasets/"
testRoot = "path/to/trial/"


#
# pose data can come in using a couple of different formats.
#
# 1) jsonDir
#    Mostly we expect a directory of .json files per camera view,
#    where the filename specifies the frame number, and the .json
#    files are formatted as per OpenPose.
# 
# 2) dlccsv
#    We also have the option to read pose from csv files as output by
#    the original DeepLabCut
#
# 
poseDataType = "jsonDir"


#
# Various pose detectors will output in OpenPose format, and the number of 
# joints/keypoints output by those detectors can vary widely.
# As such, we require that you specify a "skeleton" configuration file
# to detail which keypoints to use, what names to give the result, and 
# what the hierarchy of those keypoints/joints is.
#
skeletonFile = "../open.skel.cfg"



#
# Specify the raw pose detection sources themselves
#
poseSources = (
                 "openpose_output_00/",
                 "openpose_output_02/",
                 "openpose_output_04/"
              );


#
# Then provide information about the image sources and calibration.
# Note that the image sources are only used when visualisation is enabled,
# and are optional, but the calibration sources are obligatory
# 
imgSources = (
               "00.mp4",
               "02.mp4",
               "04.mp4"
             );

calibFiles = (
               "../calib_00/00.mp4.calib",
               "../calib_00/02.mp4.calib",
               "../calib_00/04.mp4.calib"
             );




              
#
# This file is the _output_ of trackSparsePoses
# It is thus _input_ to fuseSparsePoses.
#
# It contains the cross-camera person associations
#              
assocFile = "op.trk";






# ----------------------
# Occupancy Map - best to assume you have to use z-up, but it should work with y-up or x-up if you are so inclined
# Refer to occupancy map documentation if none of this makes sense
# ----------------------

# floor area observed
minX =  1500.0;
maxX = 24000.0;
minY = -2500.0;
maxY = 12500.0;

cellSize    =  100.0;
upDir       =    "z";
planeLow    =    0.0;
planeHigh   = 2000.0;
cellPadding =   10.0;

minVisibility = 3;
useVisibility = true;
detectionThreshold = 0.8;


# ----------------------
# Occupancy Tracking
# Refer to occupancy tracking documentation for details
# ----------------------
distanceThreshold = 200.0
numNearPeaks      = 50;




# ----------------------
# Fusion
# ----------------------

# We aim to save the fused pose into a .c3d file, so best to leave this 
# always here as "true" - but there's also a plain text output that we don't 
# really use anymore.
saveC3D = true;

# We might want to align the .c3d file with some other mocap .c3d file.
# As such, we need a file which specifies the offset between the video
# and mocap data.
C3DOffsetFile = "frameOffset";

# This is the directory where we will write our output files.
# We will have one .c3d file for every person tracked and specified 
# in the assocFile
reconDir = "op-fused";

# How should we resolve the left-right labelling of the fused keypoints?
# "votes"      : Use the voting approach
# "yplanepos"  : person runs parallel to y-axis and towards +y (+x to their right)
# "yplaneneg"  : person runs parallel to y-axis and towards -y (+x to their left)
# "xplanepos"  : person runs parallel to x-axis and towards +x (+y to their left)
# "xplaneneg"  : person runs parallel to x-axis and towards -x (+y to their right)
leftRightDecision = "votes";
firstFrame = 0;

# What is the minimum confidence for a keypoint detection
# (beneath which it is ignored)?
minJointConfidence = 0.2;

# What is the inlier distance for the RANSAC process used on
# reconstruction of single keypoints?
singleJointDistanceThreshold = 100.0;

# What is the minumum number of inliers to accept a single joint 
# reconstruction?
singleJointMinInliers = 3;
```


