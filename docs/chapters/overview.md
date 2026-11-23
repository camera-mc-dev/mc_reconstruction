## The Library

`mc_reconstruction` exists primarily as a means of getting 3D poses from sparse 2D pose detectors. It also gives stand alone classes for occupancy maps and a tool for tracking through occupancy space, mostly because these tools have become the hammer with which to hit various nails, but also because it is an effective way of merging information between camera views.

### Basic structure

As usual, the source of the library is contained under the `src/` directory. This is divided into 3 subdirectories:

  - `misc/` : We add a class for loading and saving motion capture data from `.c3d` files using the EZC3D library. We also provide some tools for interfacing with OpenSim
  - `tracking/`: Contains the occupancy tracker class files
  - `recon/`: Contains classes for the occupancy map, and also an experimental and older voxel hull algorithm.
    - `poseFusion:` sub directory containing all the source for sparse pose fusion algorithms and data structures.

The remaining sections of this documentation will explain the algorithms and tools. 
