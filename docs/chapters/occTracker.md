## Occupancy Tracker

Given that we have done occupancy maps, it is obviously useful to perform tracking of occupancy through a series of occupancy maps. To facilitate this we have created the `OccupancyTracker` class which can be found under `src/tracking`


### The tracking algorithm

The tracking approach is as follows.

First, you use an `OccupancyMap` class to create occupancy maps for each frame of your data. Once you have computed the map, you then provide it to the `OccupancyTracker` by calling the `AddFrame()` method.

Once you have added all the frames, you then call the `GetTracks()` method, which processes all of the frames to return tracks of possibly multiple peaks.

What then is happening for each method?

#### Adding frames

When you add a frame to the tracker, the tracker analyses the map to identify peaks in the map. This is done through a simplistic thresholding approach:

  1) threshold map
  2) compute connected components of the thresholded map
  3) fit a 2D gaussian to each connected component.

The result is that, for each frame, the tracker has a set of peaks represented by 2D gaussians (means and covariance matrices).

#### Tracking

Tracking is best described as the process of associating frames from one frame to its neighbouring frames in time.

To do this, we initialise all detection peaks as one frame tracks, which all go into a single vector. Because we do this one frame at a time, we know that peaks of the same frame are closer to each other in the vector than peaks of distance frames.

Next, we compute the "distance" between each track and each other track. To keep this sane, we only compute the distance between track `t` and its `numNearPeaks` tracks in the vector - this works because we know the order we added tracks means that tracks close in time are close in the vector. (It might have been more meaningul to put the one frame tracks in some kind of spatial data structure so that we could use fast nearest neighbours instead, or a limit on the distance in time, or something).

The "distance" between tracks is a non-trivial computation based on their distance in time, as well as in space - see the code for the actual computation.

With those initial distances computed, we now build longer tracks by merging compatible short tracks (tracklets) together. After a merge, we need to recompute the distance of an updated track with other tracks. We always merge the "nearest" tracks until we reach our distance limit.

We don't suggest that this algorithm is suitable for very long sequences (inefficient) or for sequences with very large numbers of people. However, it works.

The result is a set of tracks, where each track is the result of merging tracklets and updating the gaussian at each frame.

The tracker needs to be supplied with various settings on creation:

```cpp
	struct SOccTrackSettings
	{
		cv::Mat visMap;           // visibility map of the occupancy area.
		int minVisibility;        // minimum visibility of a peak.
		float detectionThreshold; // minimum occupancy.
		
		bool useVisibility;       // when normalising occupancy is the computation count/cameras  or count/visibility?
		int numCameras;
		
		float distanceThreshold;  // this is based on the track distance, which is spatialDistance + log( timeDistance )
		int   numNearPeaks;       // When we create the initial tracks, we compute the distance vs. the numNearPeaks 
		                          // that come after it in space and time. A small number might mean you don't connect 
		                          // the track fully. A large number will mean a lot of redundant calculation.
	};
```

## Occupancy tracking program

`mc_reconstruction` comes with an example tool for doing occupancy tracking on a set of mask images from calibrated cameras.

The app is `apps/occTrack.cpp`.

To use the tool you simply supply it with a configuration file, an example of which is:

```bash
# the usual data roots
dataRoot = "/your/data/root/"
testRoot = "your/test/root/"

# location of the mask image sources
maskDirs = ( "fg-00/", "fg-01/", "fg-02/", "fg-03/", "fg-04/" );

# calibration files for each source
calibFiles = ( "00.mp4.calib", "01.mp4.calib", "02.mp4.calib", "03.mp4.calib", "04.mp4.calib");

#
# First are settings for the occupancy map
#

# extents of the region
# remember that for z-up volumes we don't care about minZ/maxZ
minX = -5000
maxX =  5000
minY = -5000
maxY =  5000
minZ =  0
maxZ =  0

# map resolution
cellSize = 10

# orientation of the space
upDir = "z"

# plane settings. This test app allows for a single occupancy volume 
# between two height planes
planeLow  =    0.0
planeHigh = 2000.0

# should we pad the size of cells? We often gain some robustness from this.
cellPadding = 5.0


#
# now we define the settings for the tracker 
#
minVisibility = 3;
useVisibility = true;

# these will probably need to be tweaked for your data.
detectionThreshold = 0.75;
distanceThreshold = 200.0
numNearPeaks = 50


# If you have a sequence that doesn't really get started instantly,
# or for any reason, you can skip ahead a bit
firstFrame = 0

```
