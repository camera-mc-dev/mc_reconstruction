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
		starts.push_back( calibs[ ci->first ].GetCameraCentre() );
		rays.push_back( calibs[ci->first].Unproject( ci->second.joints[jc0] ) );
		confidences.push_back( ci->second.confidences[jc0] );
		cams.push_back( ci->first );
	}
}


void ReconstructSingleJoint( std::vector< Calibration > calibs, int jc0, float distanceThresh, int minNumInliers, PersonPose3D &person )
{
	// create a set of rays for this body part.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	std::vector< int > cams;
	GetRays( calibs, person, jc0, starts, rays, confidences, cams );
	
	
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




void ReconstructJointPair( std::vector< Calibration > calibs, skeleton_t skelType, int jc0, int jc1, PersonPose3D &person )
{
	//
	// The detectors have a really big problem with left and right.
	// So what do we do to resolve that?
	//
	
	// 1) put all of the detections, left and right, in our set of rays.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	std::vector< int > cams;
	GetRays( calibs, person, jc0, starts, rays, confidences, cams );
	GetRays( calibs, person, jc1, starts, rays, confidences, cams );
	
	
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
	// This is the stupid way of doing it:
	// know that we always face towards +ve y...
	//
	
	if( pa(0) > pb(0) ) // pa is right because x increases to the right.
	{
		if( IsLeft( jc0, skelType) )
		{
			person.joints[jc0] = pb;
			person.joints[jc1] = pa;
		}
		else
		{
			person.joints[jc0] = pa;
			person.joints[jc1] = pb;
		}
	}
	else // pa is left
	{
		if( IsLeft( jc0,skelType ) )
		{
			person.joints[jc0] = pa;
			person.joints[jc1] = pb;
		}
		else
		{
			person.joints[jc0] = pb;
			person.joints[jc1] = pa;
		}
	}
}
