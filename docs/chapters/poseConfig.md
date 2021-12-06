## Configuration file for poseTracker and poseFusion

You can use the same configuration file for both `trackSparsePoses` and `fuseSparsePoses`, as many of the configuration parameters are shared between the two.

```bash

#
# Start with the usual path information.
#
dataRoot = "/path/to/datasets/"
testRoot = "path/to/trial/"

#
# Then provide information about the image sources,
# the calibration, and the pose sources.
# Note that the image sources are only used when 
# visualisation is enabled.
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



poseSources = (
                 "openpose_output_00/",
                 "openpose_output_02/",
                 "openpose_output_04/"
              );
              
#
# This file is the _output_ of trackSparsePoses
# It is thus _input_ to fuseSparsePoses.
#
# It contains the cross-camera person associations
#              
assocFile = "op.trk";





# ----------------------
# Specify which sparse-pose detection algorithm was used.
# Currently support:
# "open"  : 24 part OpenPose
# "alpha" : 18 part AlphaPose 
# "dlc"   :14 part deep lab cut (old original deep lab cut)
# ----------------------
skelType = "open";



# ----------------------
# Occupancy Map - best to assume you have to use z-up.
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
# ----------------------
distanceThreshold = 200.0
numNearPeaks      = 50;


# ----------------------
# Fusion
# ----------------------

# We aim to save the fused pose into a .c3d file, so best to leave this 
# always here as "true"
saveC3D = true;

# We might want to align the .c3d file with some other mocap .c3d file.
# As such, we need a file which specifies the offset between the video
# and mocap data.
C3DOffsetFile = "frameOffset";

# This is the directory where we will write our output files.
# We will have one .c3d file for every person tracked and specified 
# in the assocFile
reconDir = "op-fused-new";

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


