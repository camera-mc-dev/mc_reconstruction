## General pose tracking

As the occupancy map approach has been reasonably successful as an approach to cross-camera associations, it made sense to also use the approach for fusing other sources of pose data - in particular, we're interested in DensePose.

DensePose gives its detections as dense segmentations, assigning to each pixel labels for (personID, foreground/background, body part, point on body part (u,v) ). For cross camera association we're interested primarily in the foreground/background labelling and personID.

We can build the occupancy map in one of two ways. 

  1) The first approach is to make a binary foreground mask and project that into the occupancy plane. This is the most traditional interpretation of making the occupancy map. More specifically, we use two xy-planes for the map, one plane high, one low. For each map cell we project the z-axis line between its high and low plane and then sum the number of foreground pixels along that projection.
  
  2) The second approach is to get a bounding box around each person blob, and build the occupancy map by projecting the cell as a bbox and checking the intersection between cell bbox and detection bbox.

The second approach is massively faster, it is also massively more flexible - it means that we can use this approach for sparse pose tracking too, if we want - only testing will tell us for sure whether this bbox approach is more robust than the representative point approach.

The output of the program is fundamentally the same as the output from the `SparsePoseTracker` and tracking uses the same `OccupancyTracker` methods. The configuration for this method is also very simillar.
