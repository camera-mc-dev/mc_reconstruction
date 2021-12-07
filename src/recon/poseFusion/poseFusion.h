#ifndef MC_POSEFUSION_H
#define MC_POSEFUSION_H

#include "math/mathTypes.h"
#include "recon/occupancy.h"
#include "SDS/opt.h"
#include "math/distances.h"
#include "tracking/occupancyTracker.h"

#include <vector>
#include <map>


#include <stdexcept>
#include <boost/filesystem.hpp>

// 
// Input structure:
//
// In any individual camera view there are detections of people,
// those detections consist of a set of 2D joint positions, normally
// with some confidence value.
//
// We also want some representative point, for example, the median
// of all the joints, with its related confidence.
//
struct PersonPose
{
	std::vector< hVec2D > joints;
	std::vector< float > confidences;
	
	hVec2D representativePoint;
	float representativeConfidence;
	
	cv::Rect representativeBB;
	
	int personID;
};


//
// Output structure:
//
// Once we've completed the fusion, we will have some set of 
// 3D points, and we will also say for each view, which 2D pose
// was used to make the reconstruction.
//
struct PersonPose3D
{
	std::map<int, hVec3D > joints;
	std::map<int,PersonPose> camPers;
};




//
// Skeletons:
//
// Each sparse pose detector will have a different skeletal structure - 
// which is to say that different joint ids can refer to different body parts.
// (we should really call them feature point ids rather than joint ids,
//  because not all of the points are joints... but anyway)
//
// We use an enumeration for each skeleton type we know about.
// There are comments at the bottom of this file showing our understanding
// of each skeleton.
//
// We will want some simple tools to tell us if a joint id is expected 
// to be on the left or right of the body, and if we expect it to be 
// a pair of joints (e.g. wrists, elbows, knees, etc...).
//
enum skeleton_t {SKEL_OPOSE, SKEL_APOSE, SKEL_DLCUT};
enum leftRightDecision_t
{
	// for the first two sets of options, we know the person is facing parallel
	// to one of the scene planes, and we know also which direction they're facing.
	LRD_XPLANE_POS, LRD_YPLANE_POS, LRD_ZPLANE_POS, // face towards +ve infinity
	LRD_XPLANE_NEG, LRD_YPLANE_NEG, LRD_ZPLANE_NEG, // face towards -ve infinity
	
	// figure out which point each ray is closest to and vote on left vs. right.
	LRD_VOTE
};

int IsPair( int jc, skeleton_t skel );
bool IsLeft( int jc, skeleton_t skel );

int SkelNumJoints( skeleton_t skel );




//
// Reading pose data. 
//
// We have tools for reading poses as output by different tools.
//

// this one is for OpenPose and AlphaPose - it reads a single file which
// is a single frame.
bool ReadPoseJSON( std::string fn, std::vector< PersonPose > &poses );

// this one also handles OpenPose and AlphaPose but looks for all 
//
void ReadPoseDirJSON( skeleton_t skelType, std::string dir, std::map< int, std::vector< PersonPose > > &poses );

// and this for DeelLabCut - it reads all the frames of a sequence.
void ReadDLC_CSV( std::string fn, std::map< int, std::vector< PersonPose > > &poses );


cv::Rect RobustBBox(
                     std::vector<float> xs,
                     std::vector<float> ys,
                     hVec2D median
                   );


//
// Once we've resolved the cross-camera problem we can then do an actual reconstruction
// of a person. 
//
void ReconstructPerson( 
                        PersonPose3D &person,
                        skeleton_t skelType,
                        std::vector< Calibration > calibs,
                        float minConf,
                        leftRightDecision_t lrd,
                        int   minInliersRANSAC,
                        float distThreshRANSAC
                      ); 
                                          
//
// Reconstructing people involves making robust estimates of their 3D joint locations. 
// This can be problematic in many ways, but some of the more tricky issues are:
//   - one or more views have bad detections
//   - one or more views swap left and right labels
//   - each camera's detection is noisy.
//
// As such, we need robust methods for reconstructing the 3D point that go beyond 
// simple 3d line intersections.
//
//

hVec3D RANSACIntersectRays3D( std::vector< hVec3D > &starts, std::vector< hVec3D > &rays, std::vector<float> &confidences, std::vector<int> &resInliers, float thresh );

void ReconstructSingleJoint( std::vector< Calibration > calibs, int jc0, float minConf, float distanceThresh, int minNumInliers, PersonPose3D &person );
void ReconstructJointPair( std::vector< Calibration > calibs, skeleton_t skelType, int jc0, int jc1, float minConf, leftRightDecision_t lrd, PersonPose3D &person );
void ResolveLeftRight( 
                       leftRightDecision_t lrd, 
                       std::vector< hVec3D > &starts,
                       std::vector< hVec3D > &rays,
                       std::vector<float> &confidences,
                       std::vector<bool> &isLeft,
                       hVec3D a,
                       hVec3D b,
                       hVec3D &left, 
                       hVec3D &right
                     );
class JointPairAgent : public SDS::Agent
{
public:
	JointPairAgent( hVec3D initp, std::vector< hVec3D > &starts, std::vector< hVec3D > &rays, std::vector< float > &confidences, std::vector<int> &cams ) :
	initp(initp), starts( starts ), rays( rays ), confidences( confidences ), cams( cams )
	{
		// group rays to cameras - TODO: should do this outside rather than once per agent but, meh.
		std::map< int, std::vector<int> > cam2rc;
		
		for( unsigned rc = 0; rc < rays.size(); ++rc )
		{
			cam2rc[ cams[rc] ].push_back(rc);
		}
		
		debug = false;
	}
	
	bool debug;
	
	virtual double EvaluatePosition()
	{
		error = 0.0f;
		
		hVec3D p0, p1;
		p0 << position[0], position[1], position[2], 1.0f;
		p1 << position[3], position[4], position[5], 1.0f;
		
		//
		// First term of the error: keep the two points apart from each other.
		//
		float d  = (p0-p1).norm();
		float e0 = std::max(0.0f, (float)-log(d/20.0f) );    // 20.0 mm selected as where the error reaches 0.
		
		//
		// Compute the distance of the rays from each point.
		//
		genMatrix D = genMatrix::Zero( 2, rays.size() );
		for( unsigned rc = 0; rc < rays.size(); ++rc )
		{
			D(0,rc) = PointRayDistance3D( p0, starts[rc], rays[rc] );
			D(1,rc) = PointRayDistance3D( p1, starts[rc], rays[rc] );
		}
		
		//
		// Norm the distances so that we are more robust to outliers. Note that as D is a distance,
		// it should never be negative.
		//
		for( unsigned rc = 0; rc < 2; ++rc )
		{
			for( unsigned cc = 0; cc < 2; ++cc )
			{
// 				// extreme
// 				D(rc,cc) = log( D(rc,cc) + 1.0f );
				
				// softer
// 				D(rc,cc) = sqrt( D(rc,cc) );
				
				// more explicit
// 				if( D(rc,cc) > 150 )
// 					D(rc,cc) = 150*150;
// 				else
// 					D(rc,cc) = D(rc,cc) * D(rc,cc);
			}
		}
		
		if( debug )
		{
			cout << D << endl;
		}
		
		//
		// Compute the error.
		//
		
		//
		// We want to make sure that only one ray from a camera contributes an error againt a point.
		// Say a camera has rays a0,a1. The distance from D(a0,p0) < D(a0,p1) and D(a1,p0) < D(a1,p1)
		// That is to say, both points pass closer to p0 than to p1. The points can't both be observations
		// of p0, so the one that is further from p0 is forced to associate with p1 instead.
		// The logic follows through if there are 3 points - in which case the third point wont get associated.
		//
		float e1 = 0.0f;
		int pc, rc;
		std::vector< std::set<int> > pc2cam(2);
		while( D.minCoeff(&pc, &rc) < 100000 )
		{
			// which camera is this ray from?
			int cc = cams[rc];
			
			// have we already had a better ray associated to point pc?
			if( pc2cam[pc].count(cc) == 0 )
			{
				// no, so we can just use this error.
				pc2cam[pc].insert( cc );
				e1 += confidences[rc] * D(pc,rc);
				
				// make sure we can't use this ray again.
				D(0,rc) = D(1,rc) = 100001;        
			}
			else
			{
				// yes, we have - so this ray can't be used with this point,
				// but maybe it can be used with another point.
				D(pc,rc) = 100001;
				
			}
		}
		
		if( debug )
		{
			cout << D << endl;
		}
		
// 		//
// 		// This should be less good... but is better?
// 		//
// 		for( unsigned rc = 0; rc < rays.size(); ++rc )
// 		{
// 			e1 += std::min( D(0,rc), D(1,rc) );
// 		}
		
// 		//
// 		// How about this instead?
// 		//
// 		std::vector<float> p0errs, p1errs, errs;
// 		for( unsigned rc = 0; rc < rays.size(); ++rc )
// 		{
// 			if( D(0,rc) < D(1,rc) )
// 			{
// 				p0errs.push_back( D(0,rc) );
// 				errs.push_back( D(0,rc) );
// 			}
// 			else
// 			{
// 				p1errs.push_back( D(1,rc) );
// 				errs.push_back( D(1,rc) );
// 			}
// 		}
// 		
// 		std::sort( p0errs.begin(), p0errs.end() );
// 		std::sort( p1errs.begin(), p1errs.end() );
// 		std::sort( errs.begin(), errs.end() );
// 		
// 		float e1 = 0.0f;
// 		if( p0errs.size() > 0 )
// 			e1 += p0errs[ p0errs.size() * 0.75 ];
// 		else
// 			e1 += 250.0f;
// 		
// 		if( p1errs.size() > 0 )
// 			e1 += p1errs[ p1errs.size()*0.75 ];
// 		else 
// 			e1 += 250.0f;
// 		
// 		if( errs.size() > 0 )
// 			e1 += errs[ errs.size()*0.75 ];
// 		else
// 			e1 += 250.0f;
		
		
		
		
		
		
		//
		// We can also add a term that kind of keeps the solutions sane by keeping p0 and p1
		// equidistant from the initial p. This makes sense because our initial p is the "intersection"
		// of all the rays, so ends up pretty much in the middle of the space of probable points
		//
		float e2 = abs( (p0-initp).norm() - (p1-initp).norm() );
		
		//
		// We can further improve the sanity by encouraging the lines p0->centre and centre->p1
		// to be parallel, or at least, mostly in the same direction. That's basically just 
		// a dot-product.
		//
		// When parallel the dot product will be 1, when perpendicular it is 0, when in opposite 
		// directions, -1. So a 1-dotp will do the job.
		//
		hVec3D p0c = initp - p0;
		hVec3D cp1 = p1 - initp;
		p0c /= p0c.norm();
		cp1 /= cp1.norm();
		float e3 = 1.0 - (p0c.dot(cp1));
		if( debug )
		{
			cout << initp.transpose() << endl;
			cout << p0.transpose() << endl;
			cout << p1.transpose() << endl;
			cout << (p0-initp).norm() << " " << (p1-initp).norm() << endl;
			cout << e0 << " " << e1 << " " << e2 << " " << e3 << endl;
		}
		error = e0 + e1 + e2 + e3;
		return error;
	}
	
protected:
	std::vector< hVec3D > &starts, &rays;
	std::vector< float > &confidences;
	std::vector<int> &cams;
	
	std::map< int, std::vector<int> > cam2rc;
	
	hVec3D initp;
};
















//
//      SKELETON (Openpose):              ||        SKELETON (Alphapose):       ||        SKELETON (DeepLabCut):
//      right  left                       ||        right  left                 ||        right   left
//                                        ||                                    ||    
//      Head:                             ||        Head:                       ||        Head:
//       17      18     Ears              ||         16      17     Ears        ||             13         "forehead"
//        15    16      Eyes              ||          14    15      Eyes        ||                         
//           00         Nose              ||             00         Nose        ||             12         "chin"
//                                        ||                                    ||    
//      Body:                             ||        Body:                       ||        Body:
//           01         Neck              ||             01         Neck        ||                        Neck
//        02    05      Shoulders         ||          02    05      Shoulders   ||          08    09      Shoulders
//        03    06      Elbows            ||          03    06      Elbows      ||          07    10      Elbows
//        04    07      Wrists            ||          04    07      Wrists      ||          06    11      Wrists
//                                        ||                                    ||    
//      Legs:                             ||        Legs:                       ||        Legs:
//        09 08 12      Hips              ||          08    11      Hips        ||          02    03      Hips
//        10    13      Knees             ||          09    12      Knees       ||          01    04      Knees
//        11    14      Ankles            ||          10    13      Ankles      ||          00    05      Ankles
//                                        ||                                    ||    
//      Feet:                             ||                                    ||    
//        24    21      Heels             ||                                    ||    
//         22  19       Big toes          ||                                    ||    
//       23      20     Little toes       ||                                    ||    

#endif
