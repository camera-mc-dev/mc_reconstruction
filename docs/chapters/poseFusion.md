## Sparse Pose Fusion

Once we have solved the cross-camera person association problem and tracked the resulting people through the scene, we can confront the problem of "fusing" the 2D detections into a 3D result.

At its most basic level, the fusion problem is one of simply reconstructing a 3D point from a set of 2D observations, where we *should* know that observations are projections of the same 3D point.

In this ideal case, we simply perform back-projection on the 2D observations using the relevant camera calibration

$$
 r_a = P^{-1}_a( p_a )
$$

where $r_a$ is the resulting ray of the back-projection (represented by the function $P^{-1}()$) of the observation from camera $a$.

Given a set of cameras ${a,b,c,d,...}$ we get a set of rays ${r_a, r_b, r_c, r_d,...}$. Due to noise in the imaging system and calibration solution, any two rays in 3D space are highly unlikely to *actually* intersect, even though they are (in theory) observations of the same point in space. The correct solution is to find the point in space minimising the distance to both rays. For multiple rays, we use the algorithm of [Slabaugh *et al*](https://www.researchgate.net/publication/2837100_Optimal_Ray_Intersection_For_Computing_3D_Points)

The observations that we reconstruct 3D points from all come from the keypoint detections of various sparse-pose-detectors. A prime example of such a detector is the OpenPose system, but also AlphaPose, versions of DeepLabCut, and a whole slew of newer algorithms as the years go by.

In practice, none of the sparse pose detection algorithms is capable of identifying the exact same 3D point representing an elbow in all possible observable orientations of the elbow (or any other keypoint they are trained to detect). Worse, the detectors are quite capable of getting the detection of any point completely wrong in one or more views, and to make life really hard, they are also highly prone to confusing left and right joints on the body.

As a result, we need a more robust method of reconstructing the 3D point given the n-view observations.

### Reconstructing a single keypoint: RANSAC

Some of the "keypoints" detected by the sparse pose system are stand-alone, and some come in left-right pairs. Due to the detectors having problems with left-right labelling, especially in certain side-profile orientations, we design a special solver for the robust solution. For the other points, we simply use RANSAC.

RANSAC is a simple algorithm. Given a model (in our case, a 3D point) which we can estimate from some minimal set of data (in our case, two rays), and a surplus of data which may contain outliers, we try to identify the set of inliers by:

  1) RANdomly SAmpling a minimal set of data (two rays) from the set of all data
  2) Making a solution from that data. (i.e. intersecting the two rays)
  3) Testing the solution against the remaining data, and counting how many data points are consistent with the solution
  4) Iterating this sampling process enough times to be confident that we have a good solution.
  5) Taking the inliers from the solution with the largest set of inliers and getting a final model solution.

In truth, RANSAC is over-kill for our problem as we rarely have enough datapoints to make it better than a brute-force search or some other robust solve - but it has the great advantage of being trivial to implement. It may be prudent to consider an alternative robust solver because:

  1) We have relatively few datapoints (RANSAC is really for when you have dozens or hundreds of datapoints)
  2) RANSAC requires us to specify an arbitrary limit on what counts as an inlier, and that single threshold might not be sane in all circumstances.


### Reconstructing paired keypoints

As stated above, the sparse pose solvers can have a problem correctly labelling left and right keypoints. Using RANSAC alone *can* lead to solutions, but it can be significantly sub-optimal.

Consider the condition where there are 5 views and each makes a sufficiently confident observation. In two of the views, the left-right labelling is opposite the other views, *but the locations of the keypoints are otherwise correct*.

If we take the observations for the left keypoint, then we have 3 consistent labels, and two inconsistent labels. The RANSAC solver will probably correctly identify the majority labels and produce a solution. But, this solution is sub-optimal. The fact is, we *do* have 2 more good observations of that left keypoint, they were just mislabelled as a right and so not used during the RANSAC solve.

When you consider how different the reported observations of (for example) a shoulder are between front views, side views, and back views, losing those two views for a frame or more can significantly impact the final position estimate.

Now, consider the same situation, but where one or more of the three "left" points is in an outlier location. Now we have a problem where there are 2 wrongly labelled points and 2 correctly labelled points on the opposite side of the body, and we don't know which is right, and RANSAC will give us a random solution.

So, to avoid throwing away information too early, we instead devise a solution that initially ignores the left-right labelling, and solves for a pair of points for a set of rays containing both the left and right labelled rays. Once we have the 2 point solution, we're then just faced with the problem of deciding what is left, and what is right.

#### Two-point solve

Consider a set of rays ${r_{c_0,o_0}, r_{c_0,o_1}, ...,  r_{c_0,o_{m0}}, r_{c_1,o_0}, ..., r_{c_1,o_m1}, ..., r_{c_n,o_mn} }$ where there are $m0, m1, m2, ..., mn$ observations from each of the $n$ views.

In an ideal world, each camera view will have 2 observations per camera view, but some views might have a single observation (and there's a horrible chance we got the data association problem wrong and ended up with more than 2, but we'll ignore that for now).

First off, we find a single intersection point of all of those rays. We assume that this will solve to a centre point somewhere approximately mid-way between the two real 3D-keypoints. This is a very reasonable assumption save for problems with outliers. NOTE: We could/should do a robust single point solve here using something like our RANSAC solver above.

Next we define an objective function which, when minimised, should result in us having 2 points at the intersections of the left and right ray subsets, without us having to explicitly identify what the line subsets are.

We then minimise this objective function using a non-linear search. Initial experience suggested that the best algorithm we have for this solve is our humble implementation of SDS. See the documentation for `mc_sds` for more information.

At the time of writing, the objective function is as follows:

  - For a state `(x0,y0,z0, x1,y1,z1)` (i.e a potential solution), we compute 4 errors:
    1) `e0`: A term to push the two points away from each other. 
    2) `e1`: A term computing the distances of the points to the camera rays
    3) `e2`: A term compelling equidistance of the points from the estimated central point (above)
    4) `e3`: A term compelling the lines from p0 to the centre, and the centre to p1, to be parallel. (This p0 and p1 should be on opposite sides of the centre point).
    
Starting from an initialisation where `p0 == p1 == pc` the search will iterate until it settles on a solution, hopefully where `p0` and `p1` correctly represent the left and right points.

#### Deciding on left and right.

We can consider many ways of resolving this, but ultimately, it is pose-dependent and hard to solve in isolation. As such, we currently allow for two approaches:

  1) Voting solve
  2) Plane based solve

In the voting solve, we do a nearest-neighbour association of rays to points. As rays are labelled left or right by the detector they can now vote to give the points their final labelling. To make this more robust, we observe that labels from front and rear view cameras are more reliable than from side views. Thus we compute the dot product between the ray and the vector `p0p1` and weight the rays' votes based on the result, such that the more perpendicular the two vectors are, the stronger the vote.

In the second case, we might know that the person is always facing parallel to some scene plane: e.g. always running along a track aligned with the scene y-axis. In that case we can make a reasonable inference about what is left and right based on the relative positions of the points and the specified scene plane. For instance, if the person always runs towards positive y, then we know that *right* corresponds to positive x, thus, we can determine right and left based on the x-position of `p0` and `p1`. Obviously, this approach can only be sane on poses where there is not a significant amount of twist in the body.


#### Future directions

There are a number of things we could consider doing that might improve the overall solutions. Certainly, research continues on doing the fusion/reconstruction using neural networks and learning, or employing physical models to hone in on realistic solutions. The sparse pose fusions are basically 2D solves and thus lack any real spatial awareness, so it makes sense that GANs or other models that can get some modest awareness of the manifold of human pose in 3D should be able to improve overall 3d solutions.

Of course, we're left as ever with the terrible choice between modelling and measuring : use a model to *fix* the observations and you no longer get to know what it is that you are measuring - the world, or the model?




