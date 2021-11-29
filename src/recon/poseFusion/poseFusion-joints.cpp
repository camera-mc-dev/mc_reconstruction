#include "recon/poseFusion/poseFusion.h"
#include "math/intersections.h"


#include <cv.hpp>


hVec3D RANSACIntersectRays3D( std::vector< hVec3D > &starts, std::vector< hVec3D > &rays, std::vector<float> &confidences, std::vector<int> &resInliers, float thresh )
{
	//
	// If there are outliers while trying to reconstruct a point in 3D from a set of rays, then
	// they can have a serious impact on the final 3D point.
	//
	// One option for solving that problem is to use RANSAC. It is not ideal, especially 
	// if there are only a relatively small number of input rays, but it can be enough
	// in many cases.
	//
	// A 3D point can be inferred from the intersection of any 2 rays. As such, we find the 
	// "best" solution for all rays by randomly selecting one pair of rays and counting 
	// how many other rays are consistent with that solution.
	//
	// We repeat this many times to get a number of solutions.
	//
	// We take the inliers from the solution with the largest number of inliers, and we 
	// get the final 3D point from the intersection of the inlier rays.
	//
	// This is very basic RANSAC, and far more sophisticated outlier rejection algorithms
	// do exist and may produce better results.
	//
	
	
	int iter = 0;
	std::vector<int> bestInliers;
	while( iter < 200 && bestInliers.size() < rays.size() )
	{
		int i0 = rand()%rays.size();
		int i1 = i0;
		while( i0 == i1 )
			i1 = rand()%rays.size();
		
		std::vector< hVec3D > s, r;
		s.push_back( starts[i0] ); r.push_back( rays[i0] );
		s.push_back( starts[i1] ); r.push_back( rays[i1] );
		
		hVec3D p = IntersectRays( s, r );
		
		std::vector<int> inliers;
		for( unsigned i = 0; i < rays.size(); ++i )
		{
			float d = PointRayDistance3D( p, starts[i], rays[i] );
			if( d < thresh )
			{
				inliers.push_back( i );
			}
		}
		
		if( inliers.size() > bestInliers.size() )
			bestInliers = inliers;
		++iter;
	}
	
	std::vector< hVec3D > s, r;
	for( unsigned i = 0; i < bestInliers.size(); ++i )
	{
		s.push_back( starts[bestInliers[i]] ); r.push_back( rays[bestInliers[i]] );
	}
	resInliers = bestInliers;
	if( resInliers.size() < 2 )
	{
		hVec3D badRes; badRes << 0,0,0,-1;
		return badRes;
	}
	return IntersectRays( s, r );
}


// note that this will append to whatever is already in starts,rays,confidences,cams
void GetRays(
               float minConf,
               std::vector< Calibration > calibs,
               PersonPose3D &person,
               int jc0,
               std::vector< hVec3D > &starts,
               std::vector< hVec3D > &rays,
               std::vector<float> &confidences,
               std::vector<int> &cams
            )
{
	for( auto ci = person.camPers.begin(); ci != person.camPers.end(); ++ci )
	{
		if( ci->second.confidences[ jc0 ] > minConf )
		{
			starts.push_back( calibs[ ci->first ].GetCameraCentre() );
			rays.push_back( calibs[ci->first].Unproject( ci->second.joints[jc0] ) );
			confidences.push_back( ci->second.confidences[jc0] );
			cams.push_back( ci->first );
		}
	}
}


void ReconstructSingleJoint( std::vector< Calibration > calibs, int jc0, float minConf, float distanceThresh, int minNumInliers, PersonPose3D &person )
{
	// create a set of rays for this body part.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	std::vector< int > cams;
	GetRays( minConf, calibs, person, jc0, starts, rays, confidences, cams );
	
	
	if( starts.size() > 1 )
	{
		std::vector< int > inliers;
		person.joints[jc0] = RANSACIntersectRays3D( starts, rays, confidences, inliers, distanceThresh );
		if( inliers.size() < minNumInliers )
		{
			person.joints[jc0] << 0,0,0,-1;
		}
	}
}




void ReconstructJointPair( std::vector< Calibration > calibs, skeleton_t skelType, int jc0, int jc1, float minConf, leftRightDecision_t lrd, PersonPose3D &person )
{
	//
	// The detectors have a really big problem with left and right.
	// So what do we do to resolve that?
	//
	
	// 1) put all of the detections, left and right, in our set of rays.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	std::vector< int > cams;
	std::vector< bool > isLeft;
	GetRays( minConf, calibs, person, jc0, starts, rays, confidences, cams );
	int rc = 0;
	while( rc < rays.size() )
	{
		isLeft.push_back( IsLeft( jc0, skelType ) );
		++rc;
	}
	
	GetRays( minConf, calibs, person, jc1, starts, rays, confidences, cams );
	while( rc < rays.size() )
	{
		isLeft.push_back( IsLeft( jc1, skelType ) );
		++rc;
	}
	
	if( rays.size() < 4 )
	{
		cout << "need at least 4 rays for 2 point solve" << endl;
		cout << "only got " << rays.size() << " for joints " << jc0 << " and " << jc1 << endl;
		person.joints[jc0] << 0,0,0,0;
		person.joints[jc1] << 0,0,0,0;
		return;
	}
	
	//
	// Intersect all the rays - yes, left _and_ right
	//
	// We should get a point somewhere near the midpoint between the two real joints.
	//
	hVec3D p = IntersectRays( starts, rays );
	
	
	//
	// We'll use our implementation of SDS to solve for the best position of the 2 
	// joints. The stochastic nature of SDS seems to lend itself quite nicely to
	// a rather horrible search space. See the Agent's code to understand the 
	// function that we're trying to minimise here.
	//
	SDS::Optimiser sdsopt;
	std::vector<double> initPos(6), initRanges(6);
	initPos = { p(0), p(1), p(2), p(0), p(1), p(2) }; // initial solution is both joints on the mid-point.
	initRanges = {50,50,50, 50,50,50};                // initial search range of 5 cm
	
	// make the agents.
	std::vector< JointPairAgent* > jpagents;
	jpagents.assign(60, NULL );
	std::vector< SDS::Agent* > agents( jpagents.size() );
	for( unsigned ac = 0; ac < agents.size(); ++ac )
	{
		jpagents[ac] = new JointPairAgent(p, starts, rays, confidences, cams);
		agents[ac] = jpagents[ac];
	}
	
	// initialise the search.
	int bestAgent;
	sdsopt.InitialiseOpt(initPos, initRanges, agents, 0.1, 5000);
	
	// loop until we have a solution.
	do
	{
		bestAgent = sdsopt.StepOpt();
		hVec3D pa; pa << jpagents[bestAgent]->position[0], jpagents[bestAgent]->position[1], jpagents[bestAgent]->position[2], 1.0f;
		hVec3D pb; pb << jpagents[bestAgent]->position[3], jpagents[bestAgent]->position[4], jpagents[bestAgent]->position[5], 1.0f;
	}
	while( !sdsopt.CheckTerm() );
	
	hVec3D pa; pa << jpagents[bestAgent]->position[0], jpagents[bestAgent]->position[1], jpagents[bestAgent]->position[2], 1.0f;
	hVec3D pb; pb << jpagents[bestAgent]->position[3], jpagents[bestAgent]->position[4], jpagents[bestAgent]->position[5], 1.0f;
	
	
	//
	// Now we need to decide which point is on the left, and which is on the right.
	//
	hVec3D left, right;
	ResolveLeftRight( lrd, starts, rays, confidences, isLeft, pa, pb, left, right );
	
	if( IsLeft( jc0, skelType) )
	{
		person.joints[jc0] = left;
		person.joints[jc1] = right;
	}
	else
	{
		person.joints[jc0] = right;
		person.joints[jc1] = left;
	}
	
}

void ResolveLeftRightPlane( hVec3D planeNormal, hVec3D a, hVec3D b, hVec3D &left, hVec3D &right )
{
	// We don't actually care _where_ the plane is, only the orientation of the plane.
	// so we can assume the plane passes through the origin, which means we just do...
	float ad = planeNormal.dot(a);
	float bd = planeNormal.dot(b);
	
	// we assume the plane bisects the person between their eyes as they face forwards,
	// and that the plane normal points to the person's right.
	if( ad > bd )
	{
		right = a;
		left  = b;
	}
	else
	{
		right = b;
		left  = a;
	}
}

void ResolveLeftRightVote(  
                           std::vector< hVec3D > &starts,
                           std::vector< hVec3D > &rays,
                           std::vector<float> &confidences,
                           std::vector<bool> &isLeft,
                           hVec3D a,
                           hVec3D b,
                           hVec3D &left, 
                           hVec3D &right
                         )
{
	
	//
	// Each detection had some vote as to whether it was a detection of a left body part, or a right body part.
	// That information is contained inside isLeft.
	//
	// Depending how we solved for a and b, especially if we used the SDS solver, we don't have an explicit association of each 
	// detection to point a or b. 
	// 
	
	// So, here's our jobs.
	// 1) assign each ray explicitly to point a or point b. The obvious way to do this is 
	//    to decide which ray is closer to each point, but each camera view can only have 
	//    one association to point a and one to point b.
	//
	// 2) allow each ray to vote on whether a or b are left or right based on their associations.
	//    but, temper that association based on the detection confidence, but also on the angle 
	//    of the camera vs. the line between the point pairs. Obviously when an optical axis is
	//    perpendicular to the ab vector, we get much better belief that that view resolved left/
	//    right than when the camera axis is basically parallel to the ab vector. Besides which,
	//    when that angle gets narrow the left/right association above gets poor anyway.
	//
	
	
	//
	// So we start with the association.
	//
	// For now, we just use the distance ray to point and ignore any extra heuristics.
	//
	std::vector<int> votes( rays.size(), 1 ); // vote defaults to b...
	genMatrix D(2, rays.size());
	for( unsigned rc = 0; rc < rays.size(); ++rc )
	{
		D(0,rc) = PointRayDistance3D(a, starts[rc], rays[rc] );
		D(1,rc) = PointRayDistance3D(b, starts[rc], rays[rc] );
		
		if( D(0,rc) < D(1,rc) ) votes[rc] = 0; // .. but a was closer so change vote to a
	}
	
	//
	// Now vote weighting.
	//
	hVec3D ab = b - a;
	ab /= ab.norm();
	genMatrix T = genMatrix::Zero(2,2);
	for( unsigned rc = 0; rc < rays.size(); ++rc )
	{
		float w0 = confidences[rc]; // first weight.
		
		// Don't have the camera's optical axis to hand, but the ray direction
		// will serve just as well.
		//
		// Dot product gets us cos(theta) where theta is the angle between our
		// two rays. So, when the angle is 0, cos(theta) is 1. When the angle 
		// is perpendicular, cos(theta) is 0
		//
		float cosTheta = ab.dot( rays[rc] );
		
		// so the weight should be basically the opposite of that.
		float w1 = 1.0f - abs(cosTheta);
		
		// we will vote with some weight towards 
		// (a is left)    (b is left)
		// (a is right)   (b is right)
		//
		// based on what the detection was associated to and what it's own classification was.
		//
		if( isLeft[rc] )
		{
			T(0, votes[rc]) += w0*w1;
		}
		else
		{
			T(1, votes[rc]) += w0*w1;
		}
	}
	
	//
	// Annoyingly, that's 4 numbers for one decision.
	//
	
	// which one is more left?
	float aIsLeft = T(0,0) - T(1,0);
	float bIsLeft = T(0,1) - T(1,1);
	
	if( aIsLeft > bIsLeft )
	{
		left = a;
		right = b;
	}
	else
	{
		left = b;
		right = a;
	}
	
	
	return;
}

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
                     )
{
	hVec3D pn;
	switch( lrd )
	{
		// assume floor = z
		// person faces positive x
		// so plane is zx with y to the right.
		case LRD_XPLANE_POS:
			pn << 0,1,0,0;
			ResolveLeftRightPlane( pn, a, b, left, right );
			break;
		
		case LRD_YPLANE_POS:
			pn << 1,0,0,0;
			ResolveLeftRightPlane( pn, a, b, left, right );
			break;
		
		case LRD_ZPLANE_POS:
			throw std::runtime_error( "If person looks to z, hard to assume what right and left are" );
			break;
		
		case LRD_XPLANE_NEG:
			pn << 0,-1,0,0;
			ResolveLeftRightPlane( pn, a, b, left, right );
			break;
		
		case LRD_YPLANE_NEG:
			pn << -1,0,0,0;
			ResolveLeftRightPlane( pn, a, b, left, right );
			break;
		
		case LRD_ZPLANE_NEG:
			throw std::runtime_error( "If person looks to -z, hard to assume what right and left are" );
			break;
			
		
		case LRD_VOTE:
			ResolveLeftRightVote( starts, rays, confidences, isLeft, a, b, left, right );
			break;
	}
}
