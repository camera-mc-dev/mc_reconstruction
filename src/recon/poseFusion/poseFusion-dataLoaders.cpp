#include "recon/poseFusion/poseFusion.h"
#include <cv.hpp>

#include "misc/tokeniser.h"

int FNoFromJSONFilename( skeleton_t skelType, boost::filesystem::path pth )
{
	int num = -1;
	std::string fn = pth.filename().string();
	
	if( skelType == SKEL_OPOSE || skelType == SKEL_APOSE )
	{
		// we normally have a filename something like xx_<number>_keypoints.json
		// so let us try...
		int b = fn.rfind("_keypoints");
		if( b != std::string::npos )
		{
			int a = fn.rfind("_", b-1);
			if( a == std::string::npos )
				a = 0;
			
			std::string numStr( fn.begin() + a + 1, fn.begin()+b );
			num = atoi( numStr.c_str() );
		}
	}
	
	if( num < 0 )
	{
		// we've not found it yet. Darn.
		cout << "Don't know how to get a frame number from a filename of: " << fn << endl;
		throw std::runtime_error("Can't parse frame number from filename.");
	}
	
	return num;
	
}

void ReadPoseDirJSON( skeleton_t skelType, std::string dir, std::map< int, std::vector< PersonPose > > &poses )
{
	std::vector< boost::filesystem::path > jsonFiles;
	
	// find the images in the directory.
	boost::filesystem::path p(dir);
	
	if( boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
	{
		boost::filesystem::directory_iterator di(p), endi;
		for( ; di != endi; ++di )
		{
			std::string s = di->path().string();
			auto e = di->path().extension();
			if( e.compare(".json") == 0 )
			{
				jsonFiles.push_back( di->path() );
			}
		}
	}
	else
	{
		throw std::runtime_error("Could not find json source directory.");
	}
	
	std::sort( jsonFiles.begin(), jsonFiles.end() );
	
	//
	// Now load each json file.
	//
	for( unsigned fc = 0; fc < jsonFiles.size(); ++fc )
	{
		//
		// Parse the filename to figure out the frame number.
		//
		int frameNo = FNoFromJSONFilename( skelType, jsonFiles[fc] );
		
		//
		// Load the data from the file.
		//
		ReadPoseJSON( jsonFiles[fc].string(), poses[frameNo] );
	}
	
	cout << p.string() << " " << jsonFiles.size() << " " << poses.size() << endl;
}


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
	
	int personID = 0;
	for( auto it = people.begin(); it != people.end(); ++it )
	{
		std::vector< float > vals;
		(*it)["pose_keypoints_2d"] >> vals;
		
		PersonPose pp;
		pp = personID;
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
		std::vector<float> xs, ys, cs;
		for( unsigned jc = 0; jc < pp.joints.size(); ++jc )
		{
			if( pp.joints[jc].norm() > 0 ) // ignore any (0,0,0) points.
			{
				xs.push_back( pp.joints[jc](0) );
				ys.push_back( pp.joints[jc](1) );
				cs.push_back( pp.confidences[jc] );
			}
		}
		
		// and back to getting the representative point - the median gives us a robust
		// option.
		std::sort( xs.begin(), xs.end() );
		std::sort( ys.begin(), ys.end() );
		std::sort( cs.begin(), cs.end() );
		
		pp.representativePoint << xs[ xs.size()/2 ], ys[ ys.size()/2 ], 1.0f;
		pp.representativeConfidence = cs[ cs.size()/2 ];
		
		pp.representativeBB = RobustBBox( xs, ys, pp.representativePoint );
		
		poses.push_back(pp);
		
		++personID;
	}
	return true;
}


void ReadDLC_CSV( std::string fn, std::map< int, std::vector< PersonPose > > &poses )
{
	poses.clear();
	
	std::ifstream infi( fn );
	if( !infi )
	{
		throw std::runtime_error("Could not open DLC csv file : " + fn );
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
		p.personID = 0;
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
		std::vector<float> xs, ys, cs;
		for( unsigned jc = 0; jc < p.joints.size(); ++jc )
		{
			if( p.joints[jc].norm() > 0 ) // ignore any (0,0,0) points.
			{
				xs.push_back( p.joints[jc](0) );
				ys.push_back( p.joints[jc](1) );
				cs.push_back( p.confidences[jc] );
			}
		}
		
		// the median gives us a robust centre.
		std::sort( xs.begin(), xs.end() );
		std::sort( ys.begin(), ys.end() );
		std::sort( cs.begin(), cs.end() );
		
		p.representativePoint << xs[ xs.size()/2 ], ys[ ys.size()/2 ], 1.0f;
		p.representativeConfidence = cs[ cs.size()/2 ];
		
		p.representativeBB = RobustBBox( xs, ys, p.representativePoint );
		
		poses[ frameNo ].push_back( p );
	}
}


cv::Rect RobustBBox(
                     std::vector<float> xs,
                     std::vector<float> ys,
                     hVec3D median
                   )
{
	// What is a robust bounding box?
	
	// The most basic bbox we can use is simply to 
	// take the extremes of the xs and ys but there might be
	// weird points in there.
	
	// So let us instead assume that we want a bbox that is centred
	// on the median of the input points. At that point, what 
	// we need to determine is how wide and tall it should be.
	
	// for that we need some basic heuristic about what is going 
	// to be good for our purposes. Our aim is to use this for 
	// cross-camera association using an OccupancyMask...
	
	// TODO:
	// I'd like to do something clever here, but right now,
	// I need to worry about the rest of the algorithm first.
	
	cv::Rect bb;
	bb.x = xs.begin();
	bb.y = ys.begin();
	bb.width  = (xs.back() - bb.x);
	bb.height = (ys.back() - bb.y);
}
