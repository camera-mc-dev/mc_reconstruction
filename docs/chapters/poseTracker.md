## Pose Tracker

There are a number of sparse pose detectors in the world now, most famously OpenPose. These are systems that take in images and then report on the positions of people and a number of keypoints on those people. Typically the keypoints are the positions of major body joints - give or take the skills of the people who annotated the training data and the ability of the algorithm to reproduce that on a testing image.

As these are all 2D systems, it behoves us to want to take advantage of multiple views and perform 3D reconstruction.

The first stage of this reconstruction process is to associate the detected people between the different camera views. And while we're at it, we may as well try to track those people through the observation space.

To do this, we make use of our occupancy map and occupancy tracker classes.

  1) Detect people using occupancy maps
  2) track using occupancy tracker
  3) project tracks back into cameras to associate tracks to detections.

This process is performed by the `trackSparsePoses` tool. The tool basically does:

  1) Load pose data from various sources.
  2) for each person in each frame, compute a representative point.
  3) compute occupancy for each frame.
  4) Track through the occupancy maps.
  5) Associate tracks back to detections.
  6) Output associations for each person.

Occupancy can be calculated in one of two ways. Each pose detection is a set of keypoints in the image. From that set of keypoints we can either compute a single representative point (a confidence weighted mean of the detected keypoints) or we can compute a bounding box around the keypoints. Occupancy for a frame can then be detected based on testing the representative point vs. the cell, or testing bounding box intersection against each cell.



The config file for `trackSparsePoses` is shared with the `fuseSparsePoses` tool. An example is available in the `mc_reconstruction/configs` directory, and shown we'll after the next section.

