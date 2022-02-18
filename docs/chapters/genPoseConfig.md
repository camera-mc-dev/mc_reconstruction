## Configuration file for general pose tracker



```bash

#
# Start with the usual path information.
#
dataRoot = "/path/to/datasets/"
testRoot = "path/to/trial/"

#
# Then detail where our data comes from.
# We must supply the calibration files.
#
calibFiles = (
            "../calib_00/00.mp4.calib",
            "../calib_00/02.mp4.calib",
            "../calib_00/04.mp4.calib"
          );


#
# Pose detections can come from _either_
# sparse pose sources:
#
poseSources = (
                 "openpose_output_00/",
                 "openpose_output_02/",
                 "openpose_output_04/"
              );

#
# _or_ from segmentation sources.
# You should get an error if you try to supply both source types.
#
segSources = (
                 "dp-res00/",
                 "dp-res02/",
                 "dp-res04/"
             );

# 
# If you use pose sources, you need to specify the skeleton type.
# (the type of detector basically).
# Specify which sparse-pose detection algorithm was used.
# Currently support:
# "open"  : 24 part OpenPose
# "alpha" : 18 part AlphaPose 
# "dlc"   :14 part deep lab cut (old original deep lab cut)
# 
skelType = "open";


#
# If you use segmentation sources, you need to specify the type of 
# segmentation data.
# Currently supported is:
# "dpose" or "densepose" : DensePose. Specifically, the dual segmentation map 
#                          that Murray engineered for Detectron2.
segType = "dpose";

#
# You can also specify whether to directly use the segmentation or to 
# use bounding boxes around person blobs instead.
#
# Using bounding boxes is substantially faster.
#
seg2Rect = true;


#
# You can supply image sources too, but those are only used for debug purposes.
#
imgSources = (
               "00.mp4",
               "02.mp4",
               "04.mp4"              
             );
#
# This file is the _output_ of trackSparsePoses
# It is thus _input_ to fuseSparsePoses.
#
# It contains the cross-camera person associations
#              
assocFile = "assocs.trk";









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


```


