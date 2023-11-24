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

Tracking is fundamentally the process of determining from a set of detections how many objects are in the scene at any one time, and associating a detection to each object at each time instant.

We've taken a somewhat simplistic approach based on the assumption that, for the type of scene we're interested in, the objects of interest tend to stay in the scene for the full duration.

With that in mind, we first try to work out how many objects are in the scene. We do this by assuming that most of the time the number of detections is the same as the number of objects. Thus, we take the median of the number of detections in each frame. To give the user a bit of leeway on this, we offer a config setting: `numTracksGuide`. When set to 0.5, the user will set the number of tracks to the median number of detections per frame. Setting it to 1.0 will set it to the maximum number of detections per frame, setting it to 0 will set it to the minimum number of detections per frame.

We next identify the longest run of frames for which there are exactly `numTracks` detections, and take the middle frame of that run. We initialise the tracks on that middle frame very simply as one track to one detection. Then we simply track forwards and backwards in time, updating the position of a track by weighted mean of the detections in a frame, where the weight is based on the distance between detection and track, with each detection updating only one track.

This is fairly naive tracking but will work fine in most relevant cases.

The tracker needs to be supplied with various settings on creation:

```cpp
	struct SOccTrackSettings
	{
		cv::Mat visMap;             // visibility map of the occupancy area.
		int     minVisibility;      // minimum visibility of a peak.
		float   detectionThreshold; // minimum occupancy.
		
		bool    useVisibility;      // when normalising occupancy is the computation count/cameras  or count/visibility?
		int     numCameras;
		
		int     numTracksGuide;     // Adjust where we take the "median" from when 'guessing' how many objects there are in a trial.
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
cellSize = 50

# should we pad the size of cells? We often gain some robustness from this.
cellPadding = 15.0

# orientation of the space
upDir = "z"

# plane settings. This test app allows for a single occupancy volume 
# between two height planes
planeLow  =    0.0
planeHigh = 2000.0




#
# now we define the settings for the tracker 
#
minVisibility = 3;
useVisibility = true;

# these will probably need to be tweaked for your data.
detectionThreshold = 0.75;
numTracksGuide = 0.6;


# If you have a sequence that doesn't really get started instantly,
# or for any reason, you can skip ahead a bit
firstFrame = 0

```
