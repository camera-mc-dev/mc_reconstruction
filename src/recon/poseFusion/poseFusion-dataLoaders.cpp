#include "recon/poseFusion/poseFusion.h"
#include <cv.hpp>

bool ReadPoseJSON( std::string fn, std::vector< PersonPose > &poses )
{
	// load the correct json file.
	cv::FileStorage cvfs(fn, cv::FileStorage::READ);
	if( !cvfs.isOpened() )
	{
		cout << "Could not read file: " << fn << endl;
		return false;
	}
	
	// parse the data to get the joints of individual "people"
	poses.clear();
	cv::FileNode people = cvfs["people"];
	
	
	for( auto it = people.begin(); it != people.end(); ++it )
	{
		std::vector< float > vals;
		(*it)["pose_keypoints_2d"] >> vals;
		
		PersonPose pp;
		int kpc = 0;
		while( kpc < vals.size() )
		{
			hVec2D j;
			j << vals[kpc+0], vals[kpc+1], vals[kpc+2];	// the third value looks like a confidence, but we get away with treating it as 1.0 so, what the hey.
			pp.confidences.push_back( j(2) );
			if( j(2) > 0 )
				j(2) = 1.0f;	// leave h at 0 if not a valid point for easy ignoring..
			pp.joints.push_back(j);
			kpc += 3;
		}
		
		// while we're at it, let's create a representative point for this person.
		// This representative point is just a robust estimate of the centre of 
		// the person based on all the joints, and has various uses, not least of
		// which being for disambiguating people between camera views.
		std::vector<float> xs, ys, zs, cs;
		for( unsigned jc = 0; jc < pp.joints.size(); ++jc )
		{
			xs.push_back( pp.joints[jc](0) );
			ys.push_back( pp.joints[jc](1) );
			zs.push_back( pp.joints[jc](2) );
			cs.push_back( pp.confidences[jc] );
		}
		
		std::sort( xs.begin(), xs.end() );
		std::sort( ys.begin(), ys.end() );
		std::sort( zs.begin(), zs.end() );
		std::sort( cs.begin(), cs.end() );
		
		pp.representativePoint << xs[ xs.size()/2 ], ys[ ys.size()/2 ], zs[ zs.size()/2 ];
		pp.representativeConfidence = cs[ cs.size()/2 ];
		
		poses.push_back(pp);
	}
	return true;
}


void ReadDLC_CSV( std::string fn, std::map< int, std::vector< PersonPose > &poses )
{
	poses.clear();
	
	std::ifstream infi( fn );
	if( !infi )
	{
		throw std::runtime_error("Could not open DLC csv file : " + data.opDirs[isc] );
	}
	
	std::vector< std::vector< std::string > > lines;
	while( infi )
	{
		// read the line...
		std::string line;
		std::getline( infi, line );
		if( !infi )
			continue;
		
		// tokenise
		lines.push_back( SplitLine( line, "," ) );
	}
	
	// now we parse the lines.
	// first line doesn't matter.
	// second line is body part names (three times) - let's be lazy and assume it is always the same.
	// third line is column headers, again, let's be lazy and assume it is always the same.
	// remaining lines are relevant data.
	for( unsigned lc = 3; lc < lines.size(); ++lc )
	{
		int numJoints = (lines[lc].size() - 1) / 3;
		int frameNo = atoi(lines[lc][0].c_str() );
		PersonPose p;
		p.joints.resize( numJoints );
		p.confidences.resize( numJoints );
		int indx = 1;
		for( unsigned jc = 0; jc < numJoints; ++jc )
		{
			p.joints[jc] << atof(lines[lc][indx].c_str()), atof(lines[lc][indx+1].c_str()) ,1.0f;
			p.confidences[jc] = atof( lines[lc][indx+2].c_str() );
			indx += 3;
		}
		
		
		// while we're at it, let's create a point for synergy map computation.
		// No single specific "joint" in the data is ideal, especially as there's a
		// change that that one critical joint might be a fucked detection, wo we'll
		// consolidate across the whole detection, and get a confidence across the 
		// whole detection too.
		std::vector<float> xs, ys, zs, cs;
		for( unsigned jc = 0; jc < p.joints.size(); ++jc )
		{
			xs.push_back( p.joints[jc](0) );
			ys.push_back( p.joints[jc](1) );
			zs.push_back( p.joints[jc](2) );
			cs.push_back( p.confidences[jc] );
		}
		
		std::sort( xs.begin(), xs.end() );
		std::sort( ys.begin(), ys.end() );
		std::sort( zs.begin(), zs.end() );
		std::sort( cs.begin(), cs.end() );
		
		p.synTestPoint << xs[ xs.size()/2 ], ys[ ys.size()/2 ], zs[ zs.size()/2 ];
		p.synConfidence = cs[ cs.size()/2 ];
		
		
		poses[ frameNo ].push_back( p );
	}
}
