#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "imgio/vidsrc.h"
#include "imgio/nullsrc.h"
#include "renderer2/sdfText.h"
#include "renderer2/showImage.h"

#include "commonConfig/commonConfig.h"

#include <string>
#include <iomanip>
#include <set>
#include <sstream>

#include "libconfig.h++"

#include "math/distances.h"
#include "math/intersections.h"

#include <boost/filesystem.hpp>
#include <fstream>

#include <chrono>
#include <algorithm>

#define USE_SDS
#include "SDS/opt.h"
#include <nlopt.h>


#ifdef USE_EZC3D
#include <ezc3d/ezc3d.h>
#include <ezc3d/Header.h>
#include <ezc3d/Data.h>
#include <ezc3d/Parameter.h>
#include <ezc3d/Frame.h>
#endif


struct PersonPose
{
	std::vector< hVec2D > joints;
	std::vector< float > confidences;
	
	hVec2D synTestPoint;
	float synConfidence;
};

struct PersonPose3D
{
	std::map<int, hVec3D > joints;
	std::map<int,PersonPose> camPers;
};


// annoyingly, there is no real tracking from the openpose result,
// so we'll have to do some simple association between frames to get 
// a consistent ID of each person - which is important if we want to
// pull out any basic statistics - like ankle position/velocity/acceleration
// or knee angles, which might give us a trivial contact time detection.
// we'll assume it is enough to find the closest distance between a joint.
// we might need two joints... who knows!
struct PersonTrack
{
	// this is for frame-to-frame association.
	hVec2D j0;
	hVec2D initj0;
	
	// this is the actual pose history.
	std::map<int, PersonPose> pose;
};

std::vector< cv::Scalar > personColours2D;
std::vector< Eigen::Vector4f > personColours3D;

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
                                                                              


enum skeleton_t {OPOSE, APOSE, DLCUT};

bool IsLeft( int jc, skeleton_t skel )
{
	if( skel == OPOSE )
	{
		switch(jc)
		{
			case 18:
			case 16:
			case 5:
			case 6:
			case 7:
			case 12:
			case 13:
			case 14:
			case 21:
			case 19:
			case 20:
				return true;
				break;
		}
		return false;
	}
	else if( skel == APOSE )
	{
		switch(jc)
		{
			case 17:
			case 15:
			case 5:
			case 6:
			case 7:
			case 11:
			case 12:
			case 13:
				return true;
				break;
		}
		return false;
	}
	else if( skel == DLCUT )
	{
		switch(jc)
		{
			case 3:
			case 4:
			case 5:
			case 9:
			case 10:
			case 11:
				return true;
				break;
		}
		return false;
	}
	return false;
}

//
// For a given joint, return the opposite joint if it has one.
// i.e. if jc == left elbow, return jc of right elbow
//
int IsPair( int jc, skeleton_t skel )
{
	if( skel == OPOSE )
	{
		switch(jc)
		{
			case 17: return 18; break;   case 18: return 17; break;
			case 15: return 16; break;   case 16: return 15; break;
			
			case  5:  return 2; break;   case  2:  return 5; break;
			case  6:  return 3; break;   case  3:  return 6; break;
			case  7:  return 4; break;   case  4:  return 7; break;
			
			case 12:  return 9; break;   case  9: return 12; break;
			case 13: return 10; break;   case 10: return 13; break;
			case 14: return 11; break;   case 11: return 14; break;
			
			case 19: return 22; break;   case 22:  return 19; break;
			case 20: return 23; break;   case 23:  return 20; break;
			case 21: return 24; break;   case 24:  return 21; break;
			
		}
		return -1;
	}
	else if( skel == APOSE )
	{
		switch(jc)
		{
			case 16: return 17; break;   case 17: return 16; break;
			case 14: return 15; break;   case 15: return 14; break;
			
			case  5:  return 2; break;   case  2:  return 5; break;
			case  6:  return 3; break;   case  3:  return 6; break;
			case  7:  return 4; break;   case  4:  return 7; break;
			
			case 11: return  8; break;   case  8: return 11; break;
			case 12: return  9; break;   case  9: return 12; break;
			case 13: return 10; break;   case 10: return 13; break;
		}
		return -1;
	}
	else if( skel == DLCUT )
	{
//		// TODO!
// 		switch(jc)
// 		{
// 			
// 		}
		return -1;
	}
	return false;
}

Eigen::Vector4f leftColour, rightColour;

class PoseRenderer;



struct Data
{
	std::vector<ImageSource*> imgSources;
	std::vector<std::string> opDirs;
	std::string outDir;
	CommonConfig ccfg;
	
	std::vector< std::map< int, PersonPose > > dlcPcPoses;
	
	std::vector< std::vector< PersonPose > > pcPoses;
	std::vector< PersonPose3D > pose3D;
	std::map< int, std::map< int, PersonPose3D > > p3dhist;
	
	std::shared_ptr<PoseRenderer> ren;
	bool visualise;
	
	float rayDistanceThresh;
	float floorMinX = -250.0;
	float floorMaxX =  1500.0;
	float floorMinY = -8000.0;
	float floorMaxY =  8000.0;
	
	int cellSize;
	int cellBuffer;
	std::vector< std::vector< std::vector< cv::Rect > > > cellBBoxes;
	
	int startFrame;
	
	float synThresh = 0.75;
	int minVisibility = 3;
	float minInlierConfidence = 0.0;
	int minNumInliers = 2;
	
	std::string testRoot;
	std::string dataRoot;
	
	bool hadPersonInPrevious = false;
	cv::Point prevPersonCell;
	
	bool writeToC3D;
	int mocapOffset;
	
	skeleton_t skelType;
};


void ParseConfig(std::string fn, Data &data)
{
	
	
	libconfig::Config cfg;
	std::string dataRoot = data.ccfg.dataRoot;
	std::string testRoot;
	
	int startFrame = 0;
	
	std::vector<std::string> imgDirs, vidFiles, calibFiles;
	data.visualise = false;
	data.cellSize = 25;
	data.cellBuffer = 75;
	try
	{
		cfg.readFile(fn.c_str());
		if( cfg.exists("dataRoot") )
			dataRoot = (const char*)cfg.lookup("dataRoot");
		testRoot = (const char*)cfg.lookup("testRoot");
		data.testRoot = testRoot;
		data.dataRoot = dataRoot;
		
		if( cfg.exists("imgDirs" ) )
		{
			libconfig::Setting &idirs = cfg.lookup("imgDirs");
			for( unsigned ic = 0; ic < idirs.getLength(); ++ic )
			{
				std::string s;
				s = dataRoot + testRoot + (const char*) idirs[ic];
				imgDirs.push_back(s);
			}
		}
		
		libconfig::Setting &opdirs = cfg.lookup("opDirs");
		for( unsigned ic = 0; ic < opdirs.getLength(); ++ic )
		{
			std::string s;
			s = dataRoot + testRoot + (const char*) opdirs[ic];
			data.opDirs.push_back(s);
		}
		
		std::string odir = (const char*)cfg.lookup("outDir");
		data.outDir = dataRoot + testRoot + "/" + odir + "/";
		
		
		data.floorMaxX = cfg.lookup("floorMaxX");
		data.floorMinX = cfg.lookup("floorMinX");
		data.floorMaxY = cfg.lookup("floorMaxY");
		data.floorMinY = cfg.lookup("floorMinY");
		
		if( cfg.exists("cellSize") )
			data.cellSize = cfg.lookup("cellSize");
		if( cfg.exists("cellBuffer") )
			data.cellSize = cfg.lookup("cellBuffer");
		
		if( cfg.exists("startFrame") )
			startFrame = cfg.lookup("startFrame");
		
		if( cfg.exists("vidFiles" ) )
		{
			libconfig::Setting &vidfs = cfg.lookup("vidFiles");
			for( unsigned ic = 0; ic < vidfs.getLength(); ++ic )
			{
				std::string s;
				s = dataRoot + testRoot + (const char*) vidfs[ic];
				vidFiles.push_back(s);
			}
			
			libconfig::Setting &cfiles = cfg.lookup("calibFiles");
			for( unsigned ic = 0; ic < cfiles.getLength(); ++ic )
			{
				std::string s;
				s = dataRoot + testRoot + (const char*) cfiles[ic];
				calibFiles.push_back(s);
			}
		}
		
		data.rayDistanceThresh = cfg.lookup("rayDistanceThresh");
		data.minInlierConfidence = cfg.lookup("minInlierConfidence");
		if( cfg.exists("minNumInliers") )
		{
			data.minNumInliers = cfg.lookup("minNumInliers");
		}
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
		}
		
		data.skelType = OPOSE;
		if( cfg.exists("skelType") )
		{
			std::string s = (const char*)cfg.lookup("skelType");
			if( s.compare("open") == 0 || s.compare("openpose") == 0 )
			{
				data.skelType = OPOSE;
			}
			if( s.compare("alpha") == 0 || s.compare("alphapose") == 0 )
			{
				data.skelType = APOSE;
			}
			if( s.compare("dlc") == 0 || s.compare("deeplabcut") == 0 )
			{
				data.skelType = DLCUT;
			}
		}
		
		if( cfg.exists("saveC3D") && (bool)cfg.lookup("saveC3D"))
		{
			data.writeToC3D = true;
			
			if( cfg.exists("C3DOffsetFile") )
			{
				std::string s = dataRoot + testRoot + (const char*)cfg.lookup("C3DOffsetFile");
				
				std::ifstream infi(s);
				std::string x;
				infi >> x;
				if( x.find("extra") == std::string::npos )
					infi >> data.mocapOffset;
				else
					data.mocapOffset = 0;
			}
			else
			{
				data.mocapOffset = 0;
			}
			
		}
		
		data.synThresh = cfg.lookup("synThresh");
		
		if( cfg.exists("minVisibility") )
			data.minVisibility = cfg.lookup("minVisibility");
	}
	catch( libconfig::SettingException &e)
	{
		cout << "Setting error: " << endl;
		cout << e.what() << endl;
		cout << e.getPath() << endl;
		exit(0);
	}
	catch( libconfig::ParseException &e )
	{
		cout << "Parse error:" << endl;
		cout << e.what() << endl;
		cout << e.getError() << endl;
		cout << e.getFile() << endl;
		cout << e.getLine() << endl;
		exit(0);
	}
	
	boost::filesystem::path p(data.outDir);
	if( !boost::filesystem::exists(p) )
	{
		boost::filesystem::create_directories(p);
	}
	else
	{
		if( !boost::filesystem::is_directory(p) )
		{
			throw std::runtime_error("Output location exists, but is not a directory.");
		}
	}
	
	cout << imgDirs.size() << " " << vidFiles.size() << " " << calibFiles.size() << endl;
	assert( (imgDirs.size() > 0 ) || (vidFiles.size() == calibFiles.size() ) );
	
	if( imgDirs.size() > 0 )
	{
		for( unsigned ic = 0; ic < imgDirs.size(); ++ic )
		{
			cout << "creating source: " << imgDirs[ic] << endl;
			data.imgSources.push_back( new ImageDirectory(imgDirs[ic]));
		}
	}
	else if( vidFiles.size() > 0 )
	{
		if( data.visualise )
		{
			for( unsigned ic = 0; ic < vidFiles.size(); ++ic )
			{
				cout << "creating source: " << vidFiles[ic] << " with " << calibFiles[ic] << endl;
				ImageSource *v = new VideoSource(vidFiles[ic], calibFiles[ic]);
				data.imgSources.push_back( v );
			}
		}
		else
		{
			cout << "Not visualising, so just using a null source with video calibration files" << endl;
			for( unsigned ic = 0; ic < vidFiles.size(); ++ic )
			{
				cout << "creating null source instead of video: " << vidFiles[ic] << " with " << calibFiles[ic] << endl;
				ImageSource *v = new NullSource(calibFiles[ic]);
				data.imgSources.push_back( v );
			}
		}
	}
	else
	{
		cout << "no imgDirs nor vidFiles." << endl;
	}
	
	for( unsigned ic = 0; ic < data.imgSources.size(); ++ic )
	{
		data.imgSources[ic]->JumpToFrame( startFrame );
	}
	data.startFrame = startFrame;
	
	assert( data.opDirs.size() == data.imgSources.size() );
}

void PreComputeCellBBoxes( Data &data )
{
	int ncy, ncx;
	ncx = (data.floorMaxX-data.floorMinX)/data.cellSize;
	ncy = (data.floorMaxY-data.floorMinY)/data.cellSize;
	data.cellBBoxes.resize( ncy );
	for( unsigned yc = 0; yc < ncy; ++yc )
	{
		data.cellBBoxes[yc].resize( ncx );
		for( unsigned xc = 0; xc < ncx; ++xc )
		{
			data.cellBBoxes[yc][xc].resize( data.imgSources.size() ); 
		}
	}
	
	for( unsigned yc = 0; yc < ncy; ++yc )
	{
		for( unsigned xc = 0; xc < ncx; ++xc )
		{
			for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			{
				auto calib = data.imgSources[cc]->GetCalibration();
				
				float tmpv;
				float miny, minx;
				float maxy, maxx;
				float minz, maxz;
				
				tmpv = data.floorMinY + data.cellSize * (yc + 0.5f);
				miny = tmpv - data.cellSize/2.0 - data.cellBuffer;
				maxy = tmpv + data.cellSize/2.0 + data.cellBuffer;
				
				tmpv = data.floorMinX + data.cellSize * (xc + 0.5f);
				minx = tmpv - data.cellSize/2.0 - data.cellBuffer;
				maxx = tmpv + data.cellSize/2.0 + data.cellBuffer;
				
				minz = 500;
				maxz = 2400;
				
				float bbx, bbX, bby, bbY;
				bbx = calib.width;
				bby = calib.height;
				bbX = bbY = 0;
				
				hVec3D p3; p3 << minx, miny, minz, 1.0f;
				hVec2D p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << minx, maxy, minz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << maxx, maxy, minz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << maxx, miny, minz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << minx, miny, maxz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << minx, maxy, maxz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << maxx, maxy, maxz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				p3 << maxx, miny, maxz, 1.0f;
				p2 = calib.Project(p3);
				bbx = std::min( p2(0), bbx );
				bby = std::min( p2(1), bby );
				bbX = std::max( p2(0), bbX );
				bbY = std::max( p2(1), bbY );
				
				data.cellBBoxes[yc][xc][cc] =  cv::Rect( bbx, bby, bbX-bbx, bbY-bby );
			}
		}
	}
}

bool ReadPoses( std::string fn, std::vector< PersonPose > &poses )
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
		
		// while we're at it, let's create a point for synergy map computation.
		// No single specific "joint" in the data is ideal, especially as there's a
		// change that that one critical joint might be a fucked detection, wo we'll
		// consolidate across the whole detection, and get a confidence across the 
		// whole detection too.
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
		
		pp.synTestPoint << xs[ xs.size()/2 ], ys[ ys.size()/2 ], zs[ zs.size()/2 ];
		pp.synConfidence = cs[ cs.size()/2 ];
		
		poses.push_back(pp);
	}
	return true;
}

hVec3D RobustIntersectRays3D( std::vector< hVec3D > &starts, std::vector< hVec3D > &rays, std::vector<float> &confidences, std::vector<int> &resInliers, float thresh )
{
	// If we just call intersect-rays we can have problems if, for example, the feet or legs are swapped (which is always
	// a possibility) on one or more view (too many views and we just can't know... if that happens very often then we have to
	// ignore the left/right association and find something more powerful.... which is harder but possible).
	
	// we could probably enumerate all the options just as quicky as by doing RANSAC, but... meh...
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


void PredictPeople( Data &data, std::vector< std::vector< std::pair<int,int> > > &pgroups )
{
	//
	// If there are multiple people spread throughout our capture volume then we need to know which
	// person from which view corresponds to which person from which other view.
	// We will base this decision on the neck joint.
	//
	
	auto t0 = std::chrono::steady_clock::now();
	
	int ncy, ncx;
	ncx = (data.floorMaxX-data.floorMinX)/data.cellSize;
	ncy = (data.floorMaxY-data.floorMinY)/data.cellSize;
	
	cv::Mat synMap(ncy, ncx, CV_32FC1, cv::Scalar(0.0f) );
	cv::Mat visMap(ncy, ncx, CV_32FC1, cv::Scalar(0.0f) );
	struct camPers
	{
		int cam;
		int pers;
	};
	std::vector< std::vector< std::vector< camPers > > > persMap;
	persMap.resize( ncy );
	for( unsigned yc = 0; yc < ncy; ++yc )
	{
		persMap[yc].resize( ncx );
		for( unsigned xc = 0; xc < ncx; ++xc )
		{
			persMap[yc][xc].reserve( data.imgSources.size() );
		}
	}
	
	auto t1 = std::chrono::steady_clock::now();
	
	if( data.hadPersonInPrevious )
	{
		// suppress anything away from the person.
		for( unsigned yc = 0; yc < ncy; ++yc )
		{
			for( unsigned xc = 0; xc < ncx; ++xc )
			{
				float d = sqrt( (yc-data.prevPersonCell.y)*(yc-data.prevPersonCell.y) + (xc-data.prevPersonCell.x)*(xc-data.prevPersonCell.x) );
				float d2 = d * data.cellSize;
				float v = exp( - (d2*d2) / (3000*3000) ); 
				synMap.at<float>(yc,xc) = data.imgSources.size() * (v - 1.0);
			}
		}
	}
	
	auto t2 = std::chrono::steady_clock::now();
	double internalTimes0 = 0;
	double internalTimes1 = 0;
	double internalTimes2 = 0;
	#pragma omp parallel for
	for( unsigned yc = 0; yc < ncy; ++yc )
	{
		// a bit of parallelism goes a long way here.
		for( unsigned xc = 0; xc < ncx; ++xc )
		{
			
			//
			// there may be multiple people per camera that could contribute to
			// this cell. We increase the value of the cell only by the 
			// most confident person.
			//
			
			std::vector<float> camContribs( data.imgSources.size(), 0.0f );
			
			for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			{
				bool countedCam = false;
				//cout << " -- " << yc << " " << xc << " " << cc << endl;
				
				auto ct0 = std::chrono::steady_clock::now();
				
				int bbx, bbX, bby, bbY;
				bbx = data.cellBBoxes[yc][xc][cc].x;
				bbX = bbx + data.cellBBoxes[yc][xc][cc].width;
				bby = data.cellBBoxes[yc][xc][cc].y;
				bbY = bby + data.cellBBoxes[yc][xc][cc].height;
				auto calib = data.imgSources[cc]->GetCalibration();
				
				auto ct1 = std::chrono::steady_clock::now();
				
				
				for( unsigned pc = 0; pc < data.pcPoses[cc].size(); ++pc )
				{
					
					hVec2D jp = data.pcPoses[cc][pc].synTestPoint;
					float conf = data.pcPoses[cc][pc].synConfidence;
					if( jp(2)  > 0 && jp(0) > bbx && jp(1) > bby && jp(0) < bbX && jp(1) < bbY )
					{
						camPers cp;
						cp.cam = cc;
						cp.pers = pc;
						
						persMap[yc][xc].push_back( cp );
						
						
						if( conf > camContribs[ cc ] )
						{
							//synMap.at<float>(yc,xc) += 1.0f / data.imgSources.size();
							camContribs[cc] = conf;
						}
					}
				}
				
				// cout << " -- lc " << endl;
				if( bbX > 0 && bbx < calib.width && bbY > 0 and bby < calib.height )
					visMap.at<float>(yc,xc) += 1.0f / data.imgSources.size();
				
				auto ct2 = std::chrono::steady_clock::now();
				
				internalTimes0 += std::chrono::duration <double, std::milli> (ct1-ct0).count();
				internalTimes1 += std::chrono::duration <double, std::milli> (ct2-ct1).count();
			}
			
			for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			{
				synMap.at<float>(yc,xc) += camContribs[cc];
			}
			synMap.at<float>(yc,xc) /= data.imgSources.size();
			
		}
	}
	
	
	auto t3 = std::chrono::steady_clock::now();
	
	for( unsigned yc = 0; yc < ncy; ++yc )
	{
		for( unsigned xc = 0; xc < ncx; ++xc )
		{
			synMap.at<float>(yc,xc) /= visMap.at<float>(yc,xc);
		}
	}
	
	auto t4 = std::chrono::steady_clock::now();
	
	double m,M;
	cv::Point ml, ML;
	cv::minMaxLoc( synMap, &m, &M, &ml, &ML );
	
	
	cout << "synMap mM: " << m << " " << M << " " << ML << "    " << visMap.at<float>(ML.y,ML.x) << endl;
	cout << "         : " << M*data.imgSources.size() << "    " << visMap.at<float>(ML.y,ML.x) *data.imgSources.size() << endl;
	
	
	cv::Mat tmp(ncy, ncx, CV_32FC3, cv::Scalar(0.0f, 0.0f, 0.0f) );
	for( unsigned rc = 0; rc < ncy; ++rc )
	{
		for( unsigned cc = 0; cc < ncx; ++cc )
		{
			tmp.at< cv::Vec3f >(rc,cc)[0] = visMap.at<float>(rc,cc);
			tmp.at< cv::Vec3f >(rc,cc)[1] = synMap.at<float>(rc,cc);
			tmp.at< cv::Vec3f >(rc,cc)[2] = 0.0f;
		}
	}
	
	auto t5 = std::chrono::steady_clock::now();
	
	{
		std::stringstream fnss;
		fnss << "synMaps/" << std::setw(6) << std::setfill('0') << data.imgSources[0]->GetCurrentFrameID() << "_" << M << ".png";
		cv::Mat save = synMap * 255;
		SaveImage( save, fnss.str() );
	}
	
	auto t6 = std::chrono::steady_clock::now();
	
	
	float visThresh = data.minVisibility / (float)data.imgSources.size();
	if( synMap.at<float>(ML.y, ML.x) > data.synThresh && visMap.at<float>(ML.y, ML.x) >= visThresh  )
	{
		cout << "creating pgroup: " << ML << "(syn: " << synMap.at<float>(ML.y, ML.x) << " vis: " << visMap.at<float>(ML.y,ML.x) *data.imgSources.size() << ")" << endl;
		pgroups.resize(1);
		for( int i = 0; i < persMap[ ML.y ][ ML.x ].size(); ++i )
		{
			cout << "\t" << persMap[ ML.y ][ ML.x ][i].cam << " " << persMap[ ML.y ][ ML.x ][i].pers << endl;
			
			pgroups[0].push_back( std::pair<int,int>( persMap[ ML.y ][ ML.x ][i].cam, persMap[ ML.y ][ ML.x ][i].pers ) );
			
		}
		
		data.hadPersonInPrevious = true;
		data.prevPersonCell = ML;
		
		tmp.at< cv::Vec3f >(ML.y,ML.x)[2] = 1.0f;
	}
	else
	{
		data.hadPersonInPrevious = false;
		pgroups.clear();
	}
	
	auto t7 = std::chrono::steady_clock::now();
	
	cout << " pred t1-t0: " << std::chrono::duration <double, std::milli> (t1-t0).count() << endl;
	cout << " pred t2-t1: " << std::chrono::duration <double, std::milli> (t2-t1).count() << endl;
	cout << " pred t3-t2: " << std::chrono::duration <double, std::milli> (t3-t2).count() << endl;
	cout << "\t" << internalTimes0 << endl;
	cout << "\t" << internalTimes1 << endl;
	cout << " pred t4-t3: " << std::chrono::duration <double, std::milli> (t4-t3).count() << endl;
	cout << " pred t5-t4: " << std::chrono::duration <double, std::milli> (t5-t4).count() << endl;
	cout << " pred t6-t5: " << std::chrono::duration <double, std::milli> (t6-t5).count() << endl;
	cout << " pred t7-t6: " << std::chrono::duration <double, std::milli> (t7-t6).count() << endl;
	
	
	
	

}


void GetRays( Data &data, std::vector< std::pair<int,int> > &pgroup, int jc, std::vector< hVec3D > &starts, std::vector< hVec3D > &rays, std::vector<int> &cams, std::vector<float> &confidences )
{
	for( unsigned i = 0; i < pgroup.size(); ++ i)
	{
		int &cc0 = pgroup[i].first;
		int &pc0 = pgroup[i].second;
		
		hVec3D s = data.imgSources[ cc0 ]->GetCalibration().GetCameraCentre();
		
		hVec2D j2d = data.pcPoses[cc0][pc0].joints[jc];
		float conf = data.pcPoses[cc0][pc0].confidences[jc];
		
		if( j2d(2) > 0 && conf > data.minInlierConfidence )
		{
			confidences.push_back( conf );
			j2d(2) = 1.0f;
			
			hVec3D r = data.imgSources[cc0]->GetCalibration().Unproject( j2d );
			
			starts.push_back(s);
			rays.push_back(r);
			
			cams.push_back(cc0);
			
		}
		else
		{
			continue;
		}
	}
}


void ReconstructSingleJoint( Data &data, std::vector< std::pair<int,int> > &pgroup, int jc, PersonPose3D &newPerson, std::vector<int> &inliers )
{
	// create a set of rays for this body part.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	
	std::vector< int > cams;
	GetRays( data, pgroup, jc, starts, rays, cams, confidences );
	
	if( starts.size() > 1 )
	{
		newPerson.joints[jc] = RobustIntersectRays3D( starts, rays, confidences, inliers, data.rayDistanceThresh );
		if( inliers.size() < data.minNumInliers )
		{
			newPerson.joints[jc] << 0,0,0,-1;
		}
	}
}


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
	}
	
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
		while( D.minCoeff(&pc, &rc) < 1000 )
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
				D(0,rc) = D(1,rc) = 1001;        
			}
			else
			{
				// yes, we have - so this ray can't be used with this point,
				// but maybe it can be used with another point.
				D(pc,rc) = 1001;
				
			}
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
		
		error = e0 + e1 + e2;
		return error;
	}
	
protected:
	std::vector< hVec3D > &starts, &rays;
	std::vector< float > &confidences;
	std::vector<int> &cams;
	
	std::map< int, std::vector<int> > cam2rc;
	
	hVec3D initp;
};

double NLOptPairSolve(unsigned n, const double *x, double *grad, void *data )
{
	JointPairAgent *agent = (JointPairAgent*) data;
	
	for( unsigned pc = 0; pc < n; ++pc )
	{
		agent->position[pc] = x[pc];
	}
	if( n != 3 )
		agent->position[2] = 0;
	
	grad = NULL;
	
	agent->EvaluatePosition();
	
// 	cout << "fit Error: " << agent->error << endl;
	return agent->error;
}


void ReconstructJointPair( Data &data, std::vector< std::pair<int,int> > &pgroup, int jc0, int jc1, PersonPose3D &newPerson )
{
	//
	// The detectors have a really big problem with left and right.
	// So what do we do to resolve that?
	//
	
	// 1) put all of the detections, left and right, in our set of rays.
	std::vector< hVec3D > starts, rays;
	std::vector< float > confidences;
	std::vector<int> cams;
	GetRays( data, pgroup, jc0, starts, rays, cams, confidences );
	
	int numRaysJC0 = starts.size();
	
	GetRays( data, pgroup, jc1, starts, rays, cams, confidences );
	
// 	cout << "rays from cams: " << endl;
// 	for( unsigned cc = 0; cc < cams.size(); ++cc )
// 		cout << "\t" << cc << ":" << cams[cc] << "   ";
// 	cout << endl;
	
	hVec3D p = IntersectRays( starts, rays );
	
#ifdef USE_SDS
	SDS::Optimiser sdsopt;
	std::vector<double> initPos(6), initRanges(6);
	initPos = { p(0), p(1), p(2), p(0), p(1), p(2) };
	initRanges = {50,50,50, 50,50,50};
	
	std::vector< JointPairAgent* > jpagents;
	jpagents.assign(60, NULL );
	std::vector< SDS::Agent* > agents( jpagents.size() );
	for( unsigned ac = 0; ac < agents.size(); ++ac )
	{
		jpagents[ac] = new JointPairAgent(p, starts, rays, confidences, cams);
		agents[ac] = jpagents[ac];
	}
	
	int bestAgent;
	sdsopt.InitialiseOpt(initPos, initRanges, agents, 0.1, 5000);
	
	do
	{
		bestAgent = sdsopt.StepOpt();
		hVec3D pa; pa << jpagents[bestAgent]->position[0], jpagents[bestAgent]->position[1], jpagents[bestAgent]->position[2], 1.0f;
		hVec3D pb; pb << jpagents[bestAgent]->position[3], jpagents[bestAgent]->position[4], jpagents[bestAgent]->position[5], 1.0f;
// 		cout << jpagents[bestAgent]->error << " : " << pa.transpose() << " | " << pb.transpose() << endl;
	}
	while( !sdsopt.CheckTerm() );
	
	hVec3D pa; pa << jpagents[bestAgent]->position[0], jpagents[bestAgent]->position[1], jpagents[bestAgent]->position[2], 1.0f;
	hVec3D pb; pb << jpagents[bestAgent]->position[3], jpagents[bestAgent]->position[4], jpagents[bestAgent]->position[5], 1.0f;
#else
	
	JointPairAgent jpagent(p, starts, rays, confidences, cams);
	jpagent.position.assign(6,0);
	
	
	nlopt_opt nlopt;
		
//	nlopt = nlopt_create(NLOPT_LN_COBYLA,2);
//	nlopt = nlopt_create(NLOPT_LN_BOBYQA,2);
//	nlopt = nlopt_create(NLOPT_LN_PRAXIS,2);
	nlopt = nlopt_create(NLOPT_LN_SBPLX,6);
//	nlopt = nlopt_create(NLOPT_LN_NELDERMEAD,2);
	
	nlopt_set_min_objective(nlopt, NLOptPairSolve, &jpagent);
	nlopt_set_ftol_rel(nlopt, 0.01);
	double x[6] = { p(0), p(1), p(2), p(0), p(1), p(2) };
	
	double minf;
	double initStep[6] = {50,50,50, 50,50,50};
	nlopt_set_initial_step(nlopt, initStep );	// sets initial step size in all directions to 100mm
	auto res = nlopt_optimize(nlopt, x, &minf);
	if( res > -4 && res < 0  )
	{
		cout << res << endl;
		throw std::runtime_error("NLOpt failed");
	}
	cout << "----" << endl;
	nlopt_destroy(nlopt);
	
	hVec3D pa; pa << x[0], x[1], x[2], 1.0f;
	hVec3D pb; pb << x[3], x[4], x[5], 1.0f;
	
#endif
	
	//
	// Now we need to decide which point is on the left, and which is on the right.
	//
	// This is the stupid way of doing it:
	// know that we always face towards +ve y...
	//
	
	if( pa(0) > pb(0) ) // pa is right because x increases to the right.
	{
		if( IsLeft( jc0, data.skelType) )
		{
			newPerson.joints[jc0] = pb;
			newPerson.joints[jc1] = pa;
		}
		else
		{
			newPerson.joints[jc0] = pa;
			newPerson.joints[jc1] = pb;
		}
	}
	else // pa is left
	{
		if( IsLeft( jc0,data.skelType ) )
		{
			newPerson.joints[jc0] = pa;
			newPerson.joints[jc1] = pb;
		}
		else
		{
			newPerson.joints[jc0] = pb;
			newPerson.joints[jc1] = pa;
		}
	}
	
	
	
// 	newPerson.joints[jc0] = pa;
// 	newPerson.joints[jc1] = pb;
	
// 	cout << "i0.size: " << inliers0.size() << endl;
// 	cout << "i1.size: " << inliers1.size() << endl;
// 	
// 	
// 	if( inliers0.size() > data.minNumInliers && inliers1.size() > data.minNumInliers )
// 	{
// 		//
// 		// We can now get the two points from the inlier sets.
// 		//
// 		std::vector< hVec3D > starts0, rays0;
// 		std::vector< hVec3D > starts1, rays1;
// 		
// 		std::vector<float> confidences0, confidences1;
// 		
// 		cout << "i set 0: " << endl;
// 		for( unsigned c = 0; c < inliers0.size(); ++c )
// 		{
// 			starts0.push_back( starts[ inliers0[c] ] );  rays0.push_back( rays[ inliers0[c] ] );
// 			confidences0.push_back( confidences[ inliers0[c] ] );
// 			cout << "\t" << cams[ inliers0[c] ] << " : " <<  starts0.back().transpose() << "    |    " << rays0.back().transpose() << endl;
// 		}
// 		
// 		
// 		cout << "i set 1: " << endl;
// 		for( unsigned c = 0; c < inliers1.size(); ++c )
// 		{
// 			starts1.push_back( starts[ inliers1[c] ] );  rays1.push_back( rays[ inliers1[c] ] );
// 			confidences1.push_back( confidences[ inliers1[c] ] );
// 			cout << "\t" << cams[ inliers1[c] ] << " : " << starts1.back().transpose() << "    |    " << rays1.back().transpose() << endl;
// 		}
// 		
// 		hVec3D p0 = ip0; //IntersectRays( starts0, rays0 );
// 		hVec3D p1 = ip1; //IntersectRays( starts1, rays1 );
// 		
// 		
// // 		std::vector<int> finInliers0, finInliers1;
// // 		hVec3D p0 = RobustIntersectRays3D( starts0, rays0, confidences0, finInliers0, data.rayDistanceThresh );
// // 		hVec3D p1 = RobustIntersectRays3D( starts1, rays1, confidences1, finInliers1, data.rayDistanceThresh );
// 		
// 		cout << "p0: " << p0.transpose() << endl;
// 		cout << "p1: " << p1.transpose() << endl;
// 		
// 		
// 		//
// 		// Now that hard part is deciding which point belongs to which joint.
// 		// we'll use voting for now, but that might not be sane :/
// 		//
// 		genMatrix v = genMatrix::Zero(2,2);
// 		for( unsigned c = 0; c < inliers0.size(); ++c )
// 		{
// 			if( inliers0[c] < numRaysJC0 )
// 				v(0,0) += 1.0f;
// 			else
// 				v(0,1) += 1.0f;
// 		}
// 		for( unsigned c = 0; c < inliers1.size(); ++c )
// 		{
// 			if( inliers1[c] < numRaysJC0 )
// 				v(1,0) += 1.0f;
// 			else
// 				v(1,1) += 1.0f;
// 		}
// 		
// 		float a = v(0,0) / v(0,1);  // votes for jc0 / votes for jc1
// 		float b = v(1,0) / v(1,1);  // votes for jc0 / votes for jc1
// 		
// // 		if( a > b )
// // 		{
// // 			// set 0 is jc0
// // 			newPerson.joints[jc0] = p0;
// // 			// set 1 is jc1 
// // 			newPerson.joints[jc1] = p1;
// // 		}
// // 		else
// // 		{
// // 			// set 1 is jc0
// // 			newPerson.joints[jc0] = p1;
// // 			// set 0 is jc1 
// // 			newPerson.joints[jc1] = p0;
// // 		}
// 		
// 		if( p0(0) > p1(0) ) // p0 is right because x increases to the right.
// 		{
// 			if( IsLeft( jc0, data.skelType) )
// 			{
// 				newPerson.joints[jc0] = p1;
// 				newPerson.joints[jc1] = p0;
// 			}
// 			else
// 			{
// 				newPerson.joints[jc0] = p0;
// 				newPerson.joints[jc1] = p1;
// 			}
// 		}
// 		else // p0 is left
// 		{
// 			if( IsLeft( jc0,data.skelType ) )
// 			{
// 				newPerson.joints[jc0] = p0;
// 				newPerson.joints[jc1] = p1;
// 			}
// 			else
// 			{
// 				newPerson.joints[jc0] = p1;
// 				newPerson.joints[jc1] = p0;
// 			}
// 		}
// 		
// 	}
// 	else
// 	{
// 		newPerson.joints[jc0] << 0,0,0,-1;
// 		newPerson.joints[jc1] << 0,0,0,-1;
// 	}
	
}




void ReconstructPeople( Data &data, std::vector< std::vector< std::pair<int,int> > > &pgroups )
{
	std::vector< PersonPose3D > newPeople;
	
	for( unsigned pc = 0; pc < pgroups.size(); ++pc )
	{
		PersonPose3D newPerson;
		for( unsigned i = 0; i < pgroups[pc].size(); ++ i)
		{
			int &cc0 = pgroups[pc][i].first;
			int &pc0 = pgroups[pc][i].second;
			newPerson.camPers[ cc0 ] = data.pcPoses[cc0][pc0];
		}
		// personpose3D is:
		// hVec3D joints
		// and whatever camPers was meant to be. 
		
		// just go through each joint and reconstruct from all cameras.
		// BUT, we have to do it robustly! So it needs to be a RANSAC type thing.
		int numJoints = 25;
		if( data.skelType == OPOSE )
			numJoints = 25;
		else if( data.skelType == APOSE )
			numJoints = 18;
		else if( data.skelType == DLCUT )
			numJoints = 14;
		std::vector<bool> doneJoint( numJoints, false );
		for( unsigned jc = 0; jc < numJoints; ++jc )
// 		unsigned jc = 4;
		{
			if( doneJoint[jc] )
				continue;
			
			int pairedJoint = IsPair( jc, data.skelType );
			if( pairedJoint >= 0 )
			{
				ReconstructJointPair( data, pgroups[pc], jc, pairedJoint, newPerson );
				doneJoint[jc] = true;
				doneJoint[pairedJoint] = true;
			}
			else
			{
				std::vector<int> inliers;
				ReconstructSingleJoint( data, pgroups[pc], jc, newPerson, inliers );
				doneJoint[jc] = true;
			}
		}
		
		
		
		// check that the newPerson is inside of the valid area.
		if( newPerson.joints[1](0) > data.floorMinX && newPerson.joints[1](0) < data.floorMaxX &&
			newPerson.joints[1](1) > data.floorMinY && newPerson.joints[1](1) < data.floorMaxY    )
		{
			newPeople.push_back( newPerson );
		}
	}
	
	// try to associate to previous people if there are any
	std::vector<bool> used( newPeople.size(), false );
	if( data.pose3D.size() > 0 && newPeople.size() > 0)
	{
		genMatrix err;
		err = genMatrix::Constant( data.pose3D.size(), newPeople.size(), 1000 );
		
		for( unsigned opc = 0; opc < data.pose3D.size(); ++opc )
		{
			for( unsigned npc = 0; npc < newPeople.size(); ++npc )
			{
				err(opc,npc) = std::min( 
				                          (data.pose3D[opc].joints[1] - newPeople[npc].joints[1]).norm(),
				                          (data.pose3D[opc].joints[8] - newPeople[npc].joints[8]).norm()
				                       );
			}
		}
		
		cout << err << endl;
		
		int opc, npc;
		while( err.minCoeff(&opc, &npc) < 500 )
		{
			data.pose3D[opc] = newPeople[npc];
			used[npc] = true;
			
			data.p3dhist[opc][ data.imgSources[0]->GetCurrentFrameID() ] = newPeople[npc];
			
			
			for( unsigned rc = 0; rc < data.pose3D.size(); ++rc )
				err(rc,npc) = 501;
			for( unsigned cc = 0; cc < newPeople.size(); ++cc )
				err(opc,cc) = 501;
		}
		
		for( unsigned npc = 0; npc < used.size(); ++npc )
		{
			if( !used[npc] )
			{
				data.p3dhist[data.pose3D.size()][ data.imgSources[0]->GetCurrentFrameID() ] = newPeople[npc];
				data.pose3D.push_back( newPeople[npc] );
				
			}
		}
		
		// TODO: Get rid of untracked old people.
	}
	else
	{
		data.pose3D.insert( data.pose3D.end(), newPeople.begin(), newPeople.end() );
	}
	
	cout << data.pose3D.size() << endl;
	for( auto i = data.p3dhist.begin(); i!= data.p3dhist.end(); ++i )
	{
		cout << i->first << " " << i->second.size() << endl;
	}
}



void SaveFeet( Data &data )
{
	// which track are we using?
	// We'll take the one with most frames, but really we should look for
	// the one that moves the most on the y-axis.
	auto il = data.p3dhist.begin();
	for( auto i = data.p3dhist.begin(); i != data.p3dhist.end(); ++i )
	{
		if( i->second.size() > il->second.size() )
		{
			il = i;
		}
	}
	if( il == data.p3dhist.end() )
	{
		cout << "No people for feet!" << endl;
		return;
	}
	
	
	
	
	// now go over all the frames feeding the filter
	std::stringstream ss;
	ss << data.outDir << "/left.trk";
	std::ofstream lout( ss.str() );
	
	ss.str("");
	ss << data.outDir << "/right.trk";
	std::ofstream rout( ss.str() );
	
	
	
	//
	//   11    14      Ankles
	// 
	// Feet:
	//   24    21      Heels
	//    22  19       Big toes
	//  23      20     Little toes
	//
	std::map< int, PersonPose3D > pph = il->second;
	
	for( auto fi = pph.begin(); fi != pph.end(); ++fi )
	{
		cout << fi->first << endl;
		
		rout << fi->first << " " << fi->second.joints[11].transpose() << "       ";
		rout                     << fi->second.joints[24].transpose() << "       ";
		rout                     << fi->second.joints[22].transpose() << "       ";
		rout                     << fi->second.joints[23].transpose() << endl;
		
		
		lout << fi->first << " " << fi->second.joints[14].transpose()<< "       ";
		lout                     << fi->second.joints[21].transpose()<< "       ";
		lout                     << fi->second.joints[19].transpose()<< "       ";
		lout                     << fi->second.joints[20].transpose()<< endl;
		

	}
	
	//
	// We're going to use python to process the tracking data to get a result,
	// probably much easier to do making use of the nice filters in SciPy
	//
}

void SaveBody( Data &data )
{
	//
	// We're not actually tracking, this is an output function!
	//
	
	// which track are we using?
	// We'll take the one with most frames, but really we should look for
	// the one that moves the most on the y-axis.
	auto il = data.p3dhist.begin();
	for( auto i = data.p3dhist.begin(); i != data.p3dhist.end(); ++i )
	{
		if( i->second.size() > il->second.size() )
		{
			il = i;
		}
	}
	if( il == data.p3dhist.end() )
	{
		cout << "No people for body!" << endl;
		return;
	}

	
	
	
	
	// now go over all the frames feeding the output file
	std::stringstream ss;
	ss << data.outDir << "/joints.trk";
	std::ofstream out( ss.str() );
	
	
	std::map< int, PersonPose3D > pph = il->second;
	
	for( auto fi = pph.begin(); fi != pph.end(); ++fi )
	{
		cout << fi->first << endl;
		
		out << fi->first << "\t";
		for( auto ji = fi->second.joints.begin(); ji != fi->second.joints.end(); ++ji )
		{
			out << ji->first << " " << ji->second.transpose() << "\t";
		}
		out << endl;
	}
	
	//
	// same thing, but this time we try to write to a .c3d file....
	//
	if( data.writeToC3D )
	{
#ifdef USE_EZC3D
		ezc3d::c3d c3d;
		
		ezc3d::ParametersNS::GroupNS::Parameter pointRate("RATE");
		pointRate.set(std::vector<double>({200.0}), {1});
		c3d.parameter("POINT", pointRate);
		
		
		int numPoints = 0;
		std::map<int, std::string> pointNames;
		if( data.skelType == OPOSE )
		{
			pointNames[17] = "rEar";       pointNames[18] = "lEar";
			  pointNames[15] = "rEye";  pointNames[16] = "lEye";
			               pointNames[0] = "nose";
			               pointNames[1] = "neck";
			pointNames[2] = "rShoulder";   pointNames[5] = "lShoulder";
			pointNames[3] = "rElbow";      pointNames[6] = "lElbow";
			pointNames[4] = "rWrist";      pointNames[7] = "lWrist";
			               pointNames[8] = "midHip";
			pointNames[9] = "rHip";        pointNames[12] = "lHip";
			pointNames[10] = "rKnee";      pointNames[13] = "lKnee";
			pointNames[11] = "rAnkle";     pointNames[14] = "lAnkle";
			pointNames[24] = "rHeel";      pointNames[21] = "lHeel";
			pointNames[22] = "rBToe";      pointNames[19] = "lBToe";
			pointNames[23] = "rLToe";      pointNames[20] = "lLToe";
		}
		else if( data.skelType == APOSE )
		{
			pointNames[16] = "rEar";       pointNames[17] = "lEar";
			  pointNames[14] = "rEye";  pointNames[15] = "lEye";
			               pointNames[0] = "nose";
			               pointNames[1] = "neck";
			pointNames[2] = "rShoulder";   pointNames[5] = "lShoulder";
			pointNames[3] = "rElbow";      pointNames[6] = "lElbow";
			pointNames[4] = "rWrist";      pointNames[7] = "lWrist";

			pointNames[8] = "rHip";        pointNames[11] = "lHip";
			pointNames[9] = "rKnee";       pointNames[12] = "lKnee";
			pointNames[10] = "rAnkle";     pointNames[13] = "lAnkle";
		}
		else if( data.skelType == DLCUT )
		{
			               pointNames[13] = "forehead";
			               pointNames[12] = "chin";
			pointNames[8] = "rShoulder";   pointNames[9]  = "lShoulder";
			pointNames[7] = "rElbow";      pointNames[10] = "lElbow";
			pointNames[6] = "rWrist";      pointNames[11] = "lWrist";

			pointNames[2] = "rHip";        pointNames[3]  = "lHip";
			pointNames[1] = "rKnee";       pointNames[4]  = "lKnee";
			pointNames[0] = "rAnkle";      pointNames[5]  = "lAnkle";
		}
		
		for( auto pi = pointNames.begin(); pi != pointNames.end(); ++pi )
		{
			c3d.point(pi->second);
		}
		
		auto names = c3d.pointNames();
		std::map<std::string, int> name2idx;
		for( int nc = 0; nc < names.size(); ++nc )
		{
			cout << nc << " " << names[nc] << endl;
			name2idx[ names[nc] ] = nc;
		}
		
		//
		// If the mocap offset is positive, we need to add in some empty frames.
		//
		cout << "--- empty frames for positive offset (tagA) --- " << endl;
		int fc = 0;
		while( fc <= data.mocapOffset )
		{
			//
			// "empty" frame - a point residual of -1.0 says invalid data.
			// but we still must specify a point for all pointNames.
			//
			ezc3d::DataNS::Points3dNS::Points pts;
			for( unsigned pc = 0; pc < pointNames.size(); ++pc )
			{
				ezc3d::DataNS::Points3dNS::Point pt;
				pt.x( 0.0f );
				pt.y( 0.0f );
				pt.z( 0.0f );
				pt.residual(-1.0f);
				
				pts.point(pt, name2idx[ pointNames[pc] ] );
			}
			
			ezc3d::DataNS::Frame f;
			f.add(pts);
			c3d.frame(f);
			
			cout << fc << endl;
			++fc;
		}
		
		// add a mocap frame for all video frames.
		cout << "--- all video frames (tagB) ---" << endl;
		for( int fc2 = 0; fc2 < pph.rbegin()->first; ++fc2 )
		{
			if( fc2 + data.mocapOffset < 0 )
			{
				cout << fc << " " << fc2 << " skip ( " << fc2 + data.mocapOffset << " ) " << endl;
				continue;
			}
			
			auto fi = pph.find( fc2 );
			if( fi == pph.end() )
			{
				// "empty" frame needed.
				ezc3d::DataNS::Points3dNS::Points pts;
				for( unsigned pc = 0; pc < pointNames.size(); ++pc )
				{
					ezc3d::DataNS::Points3dNS::Point pt;
					pt.x( 0.0f );
					pt.y( 0.0f );
					pt.z( 0.0f );
					pt.residual(-1.0f);
					
					pts.point(pt, name2idx[ pointNames[pc] ] );
				}
				
				ezc3d::DataNS::Frame f;
				f.add(pts);
				c3d.frame(f);
				cout << fc << " " << fc2 << " empty " << endl;
				++fc;
			}
			else
			{
				// fill a frame structure
				ezc3d::DataNS::Frame f;
				ezc3d::DataNS::Points3dNS::Points pts;
				for( unsigned pc = 0; pc < pointNames.size(); ++pc )
				{
					ezc3d::DataNS::Points3dNS::Point pt;
					
					auto ji = fi->second.joints.find( pc );
					if( ji != fi->second.joints.end() )
					{
						pt.x( ji->second(0) );
						pt.y( ji->second(1) );
						pt.z( ji->second(2) );
						pt.residual(1.0f);
					}
					else
					{
						pt.x( 0.0f );
						pt.y( 0.0f );
						pt.z( 0.0f );
						pt.residual(-1.0f);
					}
					
					pts.point(pt, name2idx[ pointNames[pc] ] );
				}
				cout << fc << " " << fc2 << " " << pts.nbPoints() << endl;
				f.add(pts);
				// add the frame to the c3d file
				c3d.frame(f);
				++fc;
			}
		}

		
		ss.str("");
		ss << data.outDir << "/joints.c3d";
		c3d.write( ss.str() );
		
		
#else
		cout << "you asked to write to c3d, but I was compiled without ezc3d support :( " << endl;
#endif
	}
	
}



void FindFixes( Data &data )
{
	cout << "Hunting fixes" << endl;
	
	std::stringstream ss;
	ss << data.dataRoot << "/" << data.testRoot << "/jFixes";
	std::ofstream outfi(ss.str(), std::ios::app);
	outfi << data.testRoot << endl;
	// iterate over people...
	
// 	cout << data.p3dhist.size() << endl;
	for( auto pi = data.p3dhist.begin(); pi != data.p3dhist.end(); ++pi )
	{
		// iterate over frames...
// 		cout << "\t" << pi->second.size() << endl;
		for( auto fi = pi->second.begin(); fi != pi->second.end(); ++fi )
		{
			PersonPose3D pose3D = fi->second;
// 			cout << "\t\t" << pose3D.camPers.size() << endl;
			
			//
			// Now iterate over this person's joints
			//
			for( auto ji = pose3D.joints.begin(); ji != pose3D.joints.end(); ++ji )
			{
				// actually, there's only a subset of the joints we're interested in
				int jc = ji->first;
				if( jc == 11 || jc == 14 || jc >= 20 )
				{
					
					
					// now iterate over the cameras
					for( auto cpi = pose3D.camPers.begin(); cpi != pose3D.camPers.end(); ++cpi )
					{
						// project this 3D joint back to the camera...
						hVec2D j2d = data.imgSources[ cpi->first ]->GetCalibration().Project( ji->second );
						
						
						// now find the 2D joint of the relevant openpose detection in this camera.
						PersonPose &pp2d = cpi->second;
						hVec2D j2dop = pp2d.joints[ ji->first ];
						
						hVec2D d = j2d - j2dop;
						
						if( d.norm() > 60 && j2dop(2) != 0)
						{
							outfi << pi->first << " " << fi->first << " " << ji->first << " " << cpi->first << " " << d.norm() << " : " << j2d.transpose() << " : " << j2dop.transpose() << endl;
						}
					}
				}
			}
			
			
		}
	}
	
	outfi << endl << endl << endl << endl;
}



class PoseRenderer: public Rendering::BasicRenderer
{
public:
	friend class Rendering::RendererFactory;
	
	unsigned currentCam;
	bool paused, advance, regress, camChanged, initialised;
	Data *data;
	
	
	bool Step()
	{
		win.setActive();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Render();

		bool needUpdate = false;
		sf::Event ev;
		while( win.pollEvent(ev) )
		{
			if( ev.type == sf::Event::Closed )
				exit(0);
			
			if( ev.type == sf::Event::KeyReleased )
			{
				if (ev.key.code == sf::Keyboard::Space )
				{
					paused = !paused;
				}
				if (paused && ev.key.code == sf::Keyboard::Up )
				{
					advance = true;
					needUpdate = true;
				}
				if (paused && ev.key.code == sf::Keyboard::Down )
				{
					regress = true;
					needUpdate = true;
				}
				
				if (paused && ev.key.code == sf::Keyboard::A )
				{
					persJoint.first = std::max(0, std::min((int)data->pose3D.size(), persJoint.first+1) );
					needUpdate = true;
				}
				if (paused && ev.key.code == sf::Keyboard::Z )
				{
					persJoint.first = std::max(0, std::min((int)data->pose3D.size(), persJoint.first-1) );
					needUpdate = true;
				}
				if (paused && ev.key.code == sf::Keyboard::S )
				{
					persJoint.second = std::max(0, std::min(18, persJoint.second+1) );
					needUpdate = true;
				}
				if (paused && ev.key.code == sf::Keyboard::X )
				{
					persJoint.second = std::max(0, std::min(18, persJoint.second-1) );
					needUpdate = true;
				}
				
				if( ev.key.code == sf::Keyboard::Left )
				{
					if( currentCam > 0 )
						--currentCam;
					else
						currentCam = data->imgSources.size()-1;
					needUpdate = true;
					cout << currentCam << endl;
				}
				if( ev.key.code == sf::Keyboard::Right )
				{
					++currentCam;
					if( currentCam == data->imgSources.size() )
						currentCam = 0;
					needUpdate = true;
					
					cout << currentCam << endl;
				}
			}
		}
		
		return needUpdate;
	}
	
	void Update()
	{
		if( !initialised )
		{
			Rendering::NodeFactory::Create( poseRoots3D, "poseRoot3D" );
			Get3dRoot()->AddChild(poseRoots3D);
			
			hVec3D o;
			o << 0,0,0,1;
			cubeMesh =  Rendering::GenerateCube(o, 50);
			cubeMesh->UploadToRenderer( smartThis );
			initialised = true;
		}
		
		Get3dCamera()->SetFromCalibration( data->imgSources[ currentCam ]->GetCalibration(), 100, 30000 );
		cv::Mat img = data->imgSources[ currentCam ]->GetCurrent().clone();
		Get2dBgCamera()->SetOrthoProjection( 0, img.cols, 0, img.rows, -10, 10 );
		
		// render the area.
		// may as well just OpenCV draw onto the image 
		hVec3D p0, p1;
		hVec2D p0i, p1i;
		p0 << data->floorMaxX, data->floorMaxY, 0.0f, 1.0f;
		p1 << data->floorMaxX, data->floorMinY, 0.0f, 1.0f;
		p0i = data->imgSources[ currentCam ]->GetCalibration().Project(p0);
		p1i = data->imgSources[ currentCam ]->GetCalibration().Project(p1);
		cv::line( img, cv::Point( p0i(0), p0i(1) ), cv::Point( p1i(0), p1i(1) ), cv::Scalar(255,255,255), 2 );
		
		p0 << data->floorMaxX, data->floorMinY, 0.0f, 1.0f;
		p1 << data->floorMinX, data->floorMinY, 0.0f, 1.0f;
		p0i = data->imgSources[ currentCam ]->GetCalibration().Project(p0);
		p1i = data->imgSources[ currentCam ]->GetCalibration().Project(p1);
		cv::line( img, cv::Point( p0i(0), p0i(1) ), cv::Point( p1i(0), p1i(1) ), cv::Scalar(255,255,255), 2 );
		
		p0 << data->floorMinX, data->floorMinY, 0.0f, 1.0f;
		p1 << data->floorMinX, data->floorMaxY, 0.0f, 1.0f;
		p0i = data->imgSources[ currentCam ]->GetCalibration().Project(p0);
		p1i = data->imgSources[ currentCam ]->GetCalibration().Project(p1);
		cv::line( img, cv::Point( p0i(0), p0i(1) ), cv::Point( p1i(0), p1i(1) ), cv::Scalar(255,255,255), 2 );
		
		p0 << data->floorMinX, data->floorMaxY, 0.0f, 1.0f;
		p1 << data->floorMaxX, data->floorMaxY, 0.0f, 1.0f;
		p0i = data->imgSources[ currentCam ]->GetCalibration().Project(p0);
		p1i = data->imgSources[ currentCam ]->GetCalibration().Project(p1);
		cv::line( img, cv::Point( p0i(0), p0i(1) ), cv::Point( p1i(0), p1i(1) ), cv::Scalar(255,255,255), 2 );
		
		// render 2d poses
		// may as well just OpenCV draw onto the image.
		for( unsigned pc = 0; pc < data->pcPoses[ currentCam ].size(); ++pc )
		{
			PersonPose &pp = data->pcPoses[ currentCam ][pc];
			if( pp.joints.size() == 0 )
				continue;
			
			for( unsigned jc = 0; jc < pp.joints.size(); ++jc )
			{
				cv::Point c; c.x = pp.joints[jc](0); c.y = pp.joints[jc](1);
				if( pp.confidences[jc] > 0 )
				{
					cv::circle( img, c, 8, personColours2D[ pc ], 3 );
					cv::Scalar confColor( 0, 255 * pp.confidences[jc], 255 - 255 * pp.confidences[jc]);
					cv::circle( img, c, 5, confColor, -1 );
				}
			}
			int a,b,c;
			switch(data->skelType)
			{
				case OPOSE: a = 2; b = 3; c = 4; break;
				case APOSE: a = 2; b = 3; c = 4; break;
				case DLCUT: a = 6; b = 7; c = 8; break;
			}
			// right arm (2-3-4) : openpose, alphapose
			//           (6-7-8) : dlcPcPoses
			if( pp.confidences[a] > 0 && pp.confidences[b] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,0,255), 5 );
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,0,0), 2 );
			}
			if( pp.confidences[b] > 0 && pp.confidences[c] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,0,255), 5 );
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,0,0), 2 );
			}
			
			// left arm (5-6-7)   : openpose, alphapose
			//          (11-10-9) : dlc
			switch(data->skelType)
			{
				case OPOSE: a = 5; b = 6; c = 7; break;
				case APOSE: a = 5; b = 6; c = 7; break;
				case DLCUT: a =11; b =10; c = 9; break;
			}
			if( pp.confidences[a] > 0 && pp.confidences[b] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,255,0), 5 );
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(255,255,255), 2 );
			}
			if( pp.confidences[b] > 0 && pp.confidences[c] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,255,0), 5 );
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(255,255,255), 2 );
			}
			
			//right leg (9-10-11) : openpose
			//          (8-9-10)  : alphapose
			//          (0-1-2 )  : dlc
			switch(data->skelType)
			{
				case OPOSE: a = 9; b = 10; c = 11; break;
				case APOSE: a = 8; b = 9; c = 10; break;
				case DLCUT: a = 0; b = 1; c = 2; break;
			}
			if( pp.confidences[a] > 0 && pp.confidences[b] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,0,255), 5 );
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,0,0), 2 );
			}
			if( pp.confidences[b] > 0 && pp.confidences[c] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,0,255), 5 );
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,0,0), 2 );
			}
			
			// left leg (12-13-14) : openpose
			//          (11-12-13) : alphapose
			//          ( 5- 4- 3) : dlc
			switch(data->skelType)
			{
				case OPOSE: a = 12; b = 13; c = 14; break;
				case APOSE: a = 11; b = 12; c = 13; break;
				case DLCUT: a = 5; b = 4; c = 3; break;
			}
			if( pp.confidences[a] > 0 && pp.confidences[b] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(0,255,0), 5 );
				cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(255,255,255), 2 );
			}
			if( pp.confidences[b] > 0 && pp.confidences[c] > 0 )
			{
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(0,255,0), 5 );
				cv::line( img, cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Point( pp.joints[c](0), pp.joints[c](1) ), cv::Scalar(255,255,255), 2 );
			}
			
// 			int a = 0;
// 			int b = ;
// 			cv::line( img, cv::Point( pp.joints[a](0), pp.joints[a](1) ), cv::Point( pp.joints[b](0), pp.joints[b](1) ), cv::Scalar(255,255,255), 2 );
		}
		
		
		
		// render 3d poses
		// TODO: Avoid creating a new person structure every update.
		poseRoots3D->Clean();
	
		Eigen::Vector4f white; white << 1,1,1,1;
		
		for( unsigned pc = 0; pc < data->pose3D.size(); ++pc )
		{
			std::shared_ptr< Rendering::SceneNode > sn;
			std::stringstream ss;
			ss << "person-root-" << pc;
			Rendering::NodeFactory::Create( sn, ss.str() );
			
			for( auto ji = data->pose3D[pc].joints.begin(); ji != data->pose3D[pc].joints.end(); ++ji )
			{
				ss.str("");
				ss << "person-" << pc << "-joint-" << ji->first;
				std::shared_ptr< Rendering::MeshNode > cn;
				Rendering::NodeFactory::Create( cn, ss.str() );
				
				cn->SetMesh( cubeMesh );
				cn->SetTexture( GetBlankTexture() );
				cn->SetShader( GetShaderProg("basicColourShader") );
				transMatrix3D T;
				T = transMatrix3D::Identity();
				T.col(3) = ji->second;
				cn->SetTransformation(T);
				
				// auto cn = Rendering::GenerateCubeNode( ji->second, 50, ss.str(), smartThis );
				if( pc == persJoint.first && ji->first == persJoint.second )
				{
					cn->SetBaseColour( white );
					
// 					auto ji2 = data->pose3D[pc].camPers.find( ji->first );
// 					if( ji2 != data->pose3D[pc].camPers.end() )
// 					{
// 						if( ji2->second.count( currentCam ) > 0 )
// 						{
// 							PersonPose &pp = data->pcPoses[ currentCam ][ data->pose3D[pc].camPers[ji->first][currentCam] ];
// 							
// 							cv::Point c; c.x = pp.joints[ji->first](0); c.y = pp.joints[ji->first](1);
// 							cv::circle( img, c, 12, cv::Scalar(255,255,255), 3 );
// 						}
// 					}
				}
				else
				{
					if( IsLeft( ji->first, data->skelType ) )
					{
						cn->SetBaseColour( leftColour );
					}
					else
					{
						cn->SetBaseColour( rightColour );
					}
					
				}
				sn->AddChild( cn );
			}
			
			poseRoots3D->AddChild(sn);
		}
		
		cv::Mat udimg = data->imgSources[ currentCam ]->GetCalibration().Undistort( img );
		SetBGImage( img );
	}
	
	std::shared_ptr< Rendering::SceneNode > poseRoots3D;
	std::shared_ptr< Rendering::Mesh > cubeMesh;
	std::pair<int,int> persJoint;
	
protected:
	PoseRenderer(unsigned width, unsigned height, std::string title) : BasicRenderer(width,height,title)
	{
		currentCam = 0;
		paused = true;
		camChanged = true;
		initialised = false;
		
		advance = regress = false;
		data = NULL;
		
		persJoint.first = 0;
		persJoint.second = 0;
	}
};




void SavePoses(Data &data, int frameNo)
{
	std::stringstream ss;
	ss << data.outDir << std::setw(8) << std::setfill('0') << frameNo << ".pose";
	
	std::ofstream outfi( ss.str() );
	outfi << data.pose3D.size() << endl;
	for( unsigned pc = 0; pc < data.pose3D.size(); ++pc )
	{
		outfi << data.pose3D[pc].joints.size() << endl;
		for( auto ji = data.pose3D[pc].joints.begin(); ji != data.pose3D[pc].joints.end(); ++ji )
		{
			outfi << ji->first << " " << ji->second.transpose() << endl;
		}
	}
}

// stolen with no guilt from stack overflow:
vector<string> SplitLine(const string& i_str, const string& i_delim)
{
	vector<string> result;
	
	size_t found = i_str.find(i_delim);
	size_t startIndex = 0;
	
	while(found != string::npos)
	{
		string temp(i_str.begin()+startIndex, i_str.begin()+found);
		result.push_back(temp);
		startIndex = found + i_delim.size();
		found = i_str.find(i_delim, startIndex);
	}
	if(startIndex != i_str.size())
		result.push_back(string(i_str.begin()+startIndex, i_str.end()));
	return result;      
}

void ReadDLC_CSV( Data &data )
{
	data.dlcPcPoses.resize( data.imgSources.size() );
	
	for( unsigned isc = 0; isc < data.imgSources.size(); ++isc )
	{
		std::ifstream infi( data.opDirs[isc] );
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
			
			
			data.dlcPcPoses[isc][ frameNo ] = p;
		}
	}
}


int main( int argc, char* argv[] )
{
	
	personColours2D.push_back( cv::Scalar(255,0,0) );     // B
	personColours2D.push_back( cv::Scalar(0,255,0) );     // G
	personColours2D.push_back( cv::Scalar(0,0,255) );     // R
	personColours2D.push_back( cv::Scalar(255,0,255) );   // M
	personColours2D.push_back( cv::Scalar(0,255,255) );   // Y
	personColours2D.push_back( cv::Scalar(255,255,0) );   // C
	personColours2D.push_back( cv::Scalar(255,255,255) ); // W
	
	personColours3D.resize( personColours2D.size() );
	personColours3D[0] << 255,0,0,1;
	personColours3D[1] << 0,255,0,1;
	personColours3D[2] << 0,0,255,1;
	personColours3D[3] << 255,0,255,1;
	personColours3D[4] << 0,255,255,1;
	personColours3D[5] << 255,255,0,1;
	personColours3D[6] << 255,255,255,1;
	
	leftColour << 0,1,0,1.0f;
	rightColour << 1,0,0,1.0f;
	
	
	
	// rather than just render the open pose result for a single camera,
	// our aim now is to move on and fuse the result of multiple cameras.
	// That should then give us a more robust result, better tracking,
	// and the ability to try and infer information about pose, or indeed,
	// feed the pose into a high-quality fitter.
	
	
	// First off, as always, parse our config.
	Data data;
	ParseConfig(argv[1], data);
	
	if( data.skelType == DLCUT )
	{
		ReadDLC_CSV( data );
	}
	
	// and initialise the renderer.
	cv::Mat img;
	img = data.imgSources[0]->GetCurrent();
	
	float winX = data.ccfg.maxSingleWindowWidth;
	float ar   = img.rows / (float)img.cols;
	float winY = winX * ar;
	if( winY > data.ccfg.maxSingleWindowHeight )
	{
		winY = data.ccfg.maxSingleWindowHeight;
		winX = winY / ar;
	}
	
	std::shared_ptr<PoseRenderer> ren;
	if( data.visualise )
	{
		Rendering::RendererFactory::Create( ren, winX, winY, "Open Pose result overlay" );
		ren->data = &data;
		data.ren = ren;
	}
	
	cout << "Precomputing cell bboxes..." << endl;
	PreComputeCellBBoxes(data);
	
	
	// now, we process each frame.
	bool done = false;
	unsigned fc = data.startFrame;
	bool update = true;
	int numFramesNoPeople = 0;
	while( !done )
	{
		cout << endl << endl << endl;
		cout << " ---------------------------------- " << endl;
		cout << fc << endl;
		auto t0 = std::chrono::steady_clock::now();
		
		data.pcPoses.clear();
		data.pcPoses.resize( data.imgSources.size() );
		
		// get pose(s) for each camera
		cout << "--- get poses " << endl;
		auto t1 = std::chrono::steady_clock::now();
		int numCamsWithData = 0;
		if( data.skelType == OPOSE || data.skelType == APOSE )
		{
			for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			{
				std::stringstream ss;
				if( data.skelType == OPOSE )
					ss << data.opDirs[cc] << std::setw(12) << std::setfill('0') << std::max((int)fc-1,0) << "_keypoints.json";
				else if( data.skelType == APOSE )
					ss << data.opDirs[cc] << fc << ".json";
				
				if(ReadPoses( ss.str(), data.pcPoses[cc] ))
					++numCamsWithData;
			}
		}
		else
		{
			for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			{
				auto i = data.dlcPcPoses[cc].find(fc);
				if( i == data.dlcPcPoses[cc].end() )
					continue;
				data.pcPoses[cc] = {i->second};
				++numCamsWithData;
			}
		}
		
		auto t2 = std::chrono::steady_clock::now();
		cout << "--- reconstruct" << endl;
		
		std::chrono::time_point<std::chrono::steady_clock> t2a, t2b;
		
		if( numCamsWithData > 1 )
		{
			cout << "  -- pred " << endl;
			// reconstruct 3D for each pose, using available cameras
// 			data.pose3D.clear();
			std::vector< std::vector< std::pair<int,int> > > pgroups;
			PredictPeople( data, pgroups );
			
			t2a = std::chrono::steady_clock::now();
			
// 			for( unsigned pc = 0; pc < pgroups.size(); ++pc )
// 			{
// 				cout << "\t[";
// 				for( unsigned i = 0; i < pgroups[pc].size(); ++i )
// 				{
// 					cout << " (" << pgroups[pc][i].first << " " << pgroups[pc][i].second << ") ";
// 				}
// 				cout << "]" << endl;
// 			}
// 			cout << endl;
			
			cout << "  -- recon" << endl;
			ReconstructPeople( data, pgroups );
			
			t2b = std::chrono::steady_clock::now();
			
			numFramesNoPeople = 0;
		}
		else
		{
			cout << " -- no folk" << endl;
			++numFramesNoPeople;
			
			if( numFramesNoPeople > 1500 )
				done = true;
			
			t2a = std::chrono::steady_clock::now();
			t2b = std::chrono::steady_clock::now();
		}
		
		auto t3 = std::chrono::steady_clock::now();
		cout << " --- render " << endl;
		
		// render result
		if( data.visualise )
		{
			if( update )
				ren->Update();
			update = ren->Step();
			while( ren->paused && !ren->advance )
			{
				if( update )
					ren->Update();
				update = ren->Step();
			}
			ren->advance = false;
			ren->regress = false;
			
// 			cv::Mat grab = ren->Capture();
// 			std::stringstream ss; ss << "poseGrabs/" << std::setw(6) << std::setfill('0') << fc << ".jpg";
// 			SaveImage( grab, ss.str() );
			
		}
// 		
// 		// save result
// 		SavePoses(data, fc);
// 		
		auto t4 = std::chrono::steady_clock::now();
		cout << "--- advance" << endl;
		// advance data sources.
		++fc;
		update = true;
		
		for( unsigned cc = 0; cc < data.imgSources.size(); ++cc )
			done = done || !data.imgSources[cc]->Advance();
		
		cout << endl << endl << endl;
		cout << "proc time: " << std::chrono::duration <double, std::milli> (t4-t0).count() << endl;
		cout << "\tprep   : " << std::chrono::duration <double, std::milli> (t1-t0).count() << endl;
		cout << "\tget    : " << std::chrono::duration <double, std::milli> (t2-t1).count() << endl;
		cout << "\trecon  : " << std::chrono::duration <double, std::milli> (t3-t2).count() << endl;
		cout << "\t\tpred:  " << std::chrono::duration <double, std::milli> (t2a-t2).count() << endl;
		cout << "\t\trecon: " << std::chrono::duration <double, std::milli> (t2b-t2a).count() << endl;
		cout << "\trender : " << std::chrono::duration <double, std::milli> (t4-t3).count() << endl;
		
// 		cout << "done? " << done << endl;
// 		if( fc > 434 )
// 			exit(0);
	}
	
	if( data.skelType == OPOSE )
	{
		cout << "save feet" << endl;
		SaveFeet(data);
	}
	cout << "save body" << endl;
	SaveBody(data);

	cout << "fixes" << endl;
	FindFixes(data);
}
	
	
