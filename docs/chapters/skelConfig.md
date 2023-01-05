## Configuration file for skeleton

Sparse pose fusion (and the various tools for rendering sparse pose detections and outputs) require that we know
the set of keypoints we want to use from the detector, and how those keypoints are related.

Furthermore, the fusion also wants to know whether keypoints can be treated as a left/right pair of keypoints,
which can help resolve problems with bad left/right labelling and just generally improve the solve.

The skeleton config file consists of two parts - a definition of the keypoints that we want to use, and a 
specification of the hierarchy of those keypoints.

```bash

#
# Specify the points within the skeleton.
# Points should come either as single elements, or as left/right pairs,
# If points are supplied as left/right pairs then the fusion algorithm
# will have extra information for resolving possible left/right swaps
#
# Each point should have a name, and the 0-indexed id from the detector.
# This example matches the OpenPose Body-25
#
points = (
             ( ("RIGHT_EAR", 17),     ("LEFT_EAR", 18) ),
             (   ("RIGHT_EYE", 15), ("LEFT_EYE", 16)   ),
             (             ("NOSE", 0 )                ),
             (             ("NECK", 1 )                ),
             (   ("RIGHT_SHO",  2), ("LEFT_SHO",  5)   ),
             ( ("RIGHT_ELBOW",  3), ("LEFT_ELBOW",  6) ),
             ( ("RIGHT_WRIST",  4), ("LEFT_WRIST",  7) ),
             (           ("MIDHIP", 8 )                ),
             (   ("RIGHT_HIP",  9), ("LEFT_HIP", 12)   ),
             (  ("RIGHT_KNEE", 10), ("LEFT_KNEE", 13)  ),
             ( ("RIGHT_ANKLE", 11), ("LEFT_ANKLE", 14) ),
             (  ("RIGHT_HEEL", 24), ("LEFT_HEEL", 21)  ),
             (  ("RIGHT_MTP1", 22), ("LEFT_MTP1", 19)  ),
             (  ("RIGHT_MTP2", 23), ("LEFT_MTP2", 20)  )
         );

#
# Using the keypoint name, indicate the _parent_ of the joint. If it 
# has no parent, then it should have _itself_ as its parent.
#
# This hierarchy is currently used only for rendering purposes.
#
hierarchy = (
              ( "RIGHT_EAR",     "RIGHT_EYE"), ( "LEFT_EAR",     "LEFT_EYE"),
              ( "RIGHT_EYE",          "NOSE"), ( "LEFT_EYE",         "NOSE"),
                                    (  "NOSE", "NECK"  ),
                                    (  "NECK", "MIDHIP"),
              ( "RIGHT_SHO",          "NECK"), ( "LEFT_SHO",         "NECK"),
              ( "RIGHT_ELBOW",   "RIGHT_SHO"), ( "LEFT_ELBOW",   "LEFT_SHO"),
              ( "RIGHT_WRIST", "RIGHT_ELBOW"), ( "LEFT_WRIST", "LEFT_ELBOW"),


                                    ("MIDHIP", "MIDHIP"),
              ( "RIGHT_HIP",        "MIDHIP"), ( "LEFT_HIP",      "MIDHIP"),
              ( "RIGHT_KNEE",    "RIGHT_HIP"), ( "LEFT_KNEE",   "LEFT_HIP"),
              ( "RIGHT_ANKLE",  "RIGHT_KNEE"), ( "LEFT_ANKLE", "LEFT_KNEE"),
              
              ( "RIGHT_HEEL",  "RIGHT_ANKLE"), ( "LEFT_HEEL", "LEFT_ANKLE"),
              ( "RIGHT_MTP1",   "RIGHT_HEEL"), ( "LEFT_MTP1",  "LEFT_HEEL"),
              ( "RIGHT_MTP2",   "RIGHT_HEEL"), ( "LEFT_MTP2",  "LEFT_HEEL")
              
            ); 
```


