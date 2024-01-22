## Occupancy Maps

At various stages of my career I've used various phrases to refer to this algorithm, including "synergy maps", but occupancy maps seems like the most meaningful title.

The concept can be traced at least as far back as the papers of Khan and Shah, such as "Tracking Multiple Occluding People by Localizing on Multiple Scene Planes" - but it does have some close relations with voxel hulls.

There are some variations in the method, and the most closely related approach to the one that tends to be used in the `mc_dev` framework is the work of Yildiz from 2012 : "A Fast Method for Tracking People with Multiple Cameras".

In general, the occupancy map approach is a way of identifying the probability of a person being present at a particular location in the scene. More specifically, we think of the scene as being representable as a 2D plane. For the most part, this is a reasonable assumption in many environments, where people (or other objects of interest) walk and move on *the ground* and don't pass above or beneath each other.

So, take the scene that you are observing, and imagine yourself being a satellite far above it looking straight down. The scene is now a flat plane with objects moving around on it. The aim of the occupancy map is to say, for each point on the *ground plane*, what the probability is of an object occupying that space. We'll make the assumption that the ground plane is the z=0 plane, and that you have made a sensible assumption about where the origin of the scene is.

Around your scene you have a number of cameras. Because you are a sensible and intelligent person, you have calibrated those cameras so that you know their projection model and you also know their position and orientation in the scene. What this means is that, for any point on the ground plane, you can project that point into all of the camera images.

Once you can project a point into an image, you can look at that part of the image and decide if that image point is within the bounds of some object of interest. We can now quite simply compute the occupancy of a ground plane point based on the sum of the camera views that observe that point as being part of an object of interest, normalised by the *visibility* of that point (which is to say, how many camera views can see that point).

In practice, we don't have to consider just the plane $z=0$, we can consider any plane $z=h$, and indeed, we can gain robustness by having our occupancy map accumulate over multiple planes. Indeed, we can go so far as to accumulate over an infinite number of planes between two finite heights - which is where Yildiz's work comes in.

Talking about a *point* on the ground plane is rather vague - we can't reasonbly compute at an infinite resolution for an infinitely large scene plane. Instead, we know that the scene is some rectangular area, and then we can decide to observe that scene at some specified resolution. Thus each "pixel" or *cell* of the occupancy map represents some finite area, be it 2 cm by 2 cm or 100 cm by 100 cm or 1 mm by 1 mm - and the choice of resolution is really a user decision.

The last decision that we have to make then is to determine just what it means to project a cell to the image, and also, what it means to test for the presence of an object in the image.

### Projecting an occupancy cell

The most basic option for this is to simply project a point at the centre of the cell into the images. This is nice and simple, and if you are doing occupancy for a plane it can be enough. However, most of the time you really want to consider the whole column of space above (and maybe even beneath) the ground plane. More generally, we can consider the cell to be a rectangular volume between two planes.

We can now project:

  1) the centre point
  2) a vertical line through the centre of the cell
  3) all 8 corner points of the cell.

In the first case, we have just a single image point at which to test the cell. In the second case, we get a line in the image to test. In the final case, we compute the smallest axis-aligned bounding box in the image that contains all the projected cell corners. We thus have a whole bounding box to test occupancy in the image.

### Testing presence

Testing for the presence of an object in the image now depends rather strongly upon the nature of the image information that we have.

As a basic rule, we state that any presence test will return 0 for the object not being present, and 1.0 for it being fully present, and we can return values inbetween for less confident expressions of presence.

The traditional thing is to consider binary foreground masks, such as produced by background subtraction algorithms or other object segmentors.

  1) test 1 point: return present or not present based on whether the pixel is foreground or not.
  2) test line: count how many pixels are foreground $n_f$ vs. the length of the line $n_p$. Return the ratio $n_f/n_l$
  3) test bbox: count how many pixels in the bbox are foreground and return the ratio vs. the area $a$ of the bbox $n_f/a$

### Identifying when objects are present.

The most basic approach to this is to simply threshold the occupancy map and take the remaining cells. You would need to perform some non-maxima suppression on that data as well.

Note that a simple threshold can be hard to get right, due to a number of factors. Firstly, detections may not be perfect meaning that cameras may report lower presence or even miss a person completely. Secondly, the nature of the algorithm is such that you can produce a phenomena known as a detection *ghost*. This is where two or more different objects have interfering projections resulting in false peaks in the occupancy map. Under ideal detections, these ghosts can be filtered out for being lower value than real peaks, but in practice, they can have peaks as large as real peaks. Managing detections ghosts is a research problem unto itself but there are ways of coping.

## The `OccupancyMap` class

The `OccupancyMap` class is defined in the source files `src/recon/occupancy.h` and `src/recon/occupancy.cpp`.

To instantiate an object, you must pass a settings structure to the constructor:

```cpp
	struct SOccMapSettings
	{
		//
		// Limits of the observation area
		//
		float minX;
		float maxX;
		float minY;
		float maxY;
		float minZ;
		float maxZ;
		
		float cellSize;
		
		//
		// What direction is "up"
		//
		upDir_t upDir;
		
		
		//
		// Observation planes (height ranges)
		//
		std::vector< SObsPlane > obsPlanes;
		
		
		//
		// calibrations
		//
		std::vector< Calibration > calibs;
		
		//
		// enlarge a cell before projection?
		// (means cells can overlap with other cells)
		//
		float cellPadding;
		
	};
```

These settings define the extents of the observation area. Note that you don't really need to worry about the extents on the direction of your up axis, as the individual scene planes will define the heights that you are measuring at. Mostly we use $z$ as our up axis, so `minZ` and `maxZ` are mostly irrelevant, but the class provides for other configurations if that is what you need.

Most of the remaining settings should be obvious, but `cellPadding` is worth explaining. When projecting a cell as a bounding box, it can be valuable to allow the cell to appear larger than its actual extents. Why would you want this? Consider if you have a detector that reports your object in your image as a single point. Now consider that the detected point's in different views do not correspond to the same point in 3D space. The result can be that when a cell is projected to the images, there may be no cell that contains the detection for all views. By padding the cell, you get some leeway without having to go to very low map resolutions.

When you create the `OccupancyMap` object, it will pre-compute all cell projections (be those point, line or bounding box) which helps with processing speed.

Computing an occupancy map for your data is then a case of calling one of the following methods:

```cpp
	// create an occupancy map by projecting cells into the masks
	// and summing along the projection line, or within the projection bb.
	// masks should be CV_8UC1 or CV_32FC1
	// maps will be CV_32FC1
	void OccupancyFromSegmentation( 
	                                std::vector< cv::Mat > &masks,
	                                proj_t projectionType, 
	                                std::vector< cv::Mat > &maps
	                              );
	
	// create an occupancy map by projecting cells into an image and 
	// checking if the input points are within the cell.
	// projection type is assumed to be bbox. 
	void OccupancyFromPoints( 
	                           std::vector< std::vector< hVec2D > > &points, 
	                           std::vector< cv::Mat > &maps
	                        );
	                        
	// create an occupancy map by projecting cells into an image and 
	// checking the overlap of projection and the provided bboxes.
	// projection type is assumed to be bbox. 
	void OccupancyFromBBoxes( 
	                           std::vector< std::vector< cv::Rect > > &bboxes, 
	                           std::vector< cv::Mat > &maps
	                        );	                        
```

The first option `OccupancyFromSegmentation` is for when you use binary segmentation masks as input, and the second option for when you have a single point detection on an object, the last for when you have your detection as a 2D bounding box.


One other method of note in the class is a simplistic tool for extracting peaks in the occupancy map:

```cpp
	void FindPeaks( 
	                cv::Mat &occ,
	                std::vector< hVec3D > oldPeaks,
	                int regionInd,
	                float exclusionRadius,
	                float minPeakValue,
	                std::vector< hVec3D > &points,
	                std::vector< Eigen::Vector3f > &peaks
	              );
```

The only special part of this algorithm is that it can use previously detected peaks to exclude the detection of new peaks. Refer to the code for specific details of this.

Equally, refer to the code for other minor utility methods, such as converting a cell point to a point in 3D space, etc.
