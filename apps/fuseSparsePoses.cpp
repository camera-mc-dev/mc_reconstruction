#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"

#include <fstream>
#include <iostream>
using std::cout;
using std::endl;


#ifdef USE_EZC3D
#include <ezc3d/ezc3d.h>
#include <ezc3d/Header.h>
#include <ezc3d/Data.h>
#include <ezc3d/Parameter.h>
#include <ezc3d/Frame.h>
#endif

//
// The job of this program is to load up sparse pose detections from 
// multiple cameras, associate the people between different views, and
// output the tracks and associations. We do not perform 3D pose reconstruction.
//
// We solve the cross-camera association problem using an occupancy map and 
// occupancy tracker. The occupancy map proposes probable locations of people
// in individual frames, and the occupancy tracker pieces together those 
// per-frame detections into tracks of multiple people. Or at least, it 
// will once we've done enough refinement on the code we're writing.
//
// The output is a simple file which lists the tracks, and the cross
// camera person associations for each frame.
// 
//
//



struct SData
{
	std::string dataRoot;
	std::string testRoot;
	
	OccupancyMap::SOccMapSettings occSettings;
	OccupancyTracker::SOccTrackSettings trackSettings;
	
	std::vector< std::string > calibFiles;
	std::vector< std::shared_ptr<ImageSource> > imgSources;
	std::vector< std::string > poseSources;
	
	// [cam][frame][person]
	std::vector< std::map< int, std::vector<PersonPose> > > pcPoses;
	
	skeleton_t skelType;
	
	
	// RANSAC settings for single joint reconstruction.
	float distanceThreshold;
	int minInliers;
	leftRightDecision_t lrd;
	
	
	float minConf;
	
	
	int firstFrame;
	bool visualise;
	
	// cross camera associations and person tracks.
	std::string assocFile;
	
	// place where we write the final 3D output.
	std::string reconDir;
	
	bool writeToC3D;
	int mocapOffset;
};

void SaveBody( std::map< int, PersonPose3D > &track, int trackNo, SData &data );

void ParseConfig( std::string cfgFile, SData &data )
{
	try
	{
		CommonConfig ccfg;
		data.dataRoot = ccfg.dataRoot;
		
		cout << "parsing config: " << cfgFile << endl;
		libconfig::Config cfg;
		cfg.readFile( cfgFile.c_str() );
		
		if( cfg.exists("dataRoot") )
			data.dataRoot = (const char*)cfg.lookup("dataRoot");
		data.testRoot = (const char*)cfg.lookup("testRoot");
		
		libconfig::Setting &calibsSetting  = cfg.lookup("calibFiles");
		libconfig::Setting &poseSrcSetting = cfg.lookup("poseSources");
		
		assert( poseSrcSetting.getLength() == calibsSetting.getLength() );
		
		data.occSettings.calibs.resize( calibsSetting.getLength() );
		for( unsigned sc = 0; sc < calibsSetting.getLength(); ++sc )
		{
			std::stringstream css;
			css << data.dataRoot << "/" << data.testRoot << "/" << (const char*) calibsSetting[sc];
			
			cout << css.str() << endl;
			
			data.occSettings.calibs[sc].Read( css.str() );
		}
		
		data.poseSources.resize( poseSrcSetting.getLength() );
		for( unsigned sc = 0; sc < poseSrcSetting.getLength(); ++sc )
		{
			std::stringstream pss;
			pss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) poseSrcSetting[sc];
			data.poseSources[sc] = pss.str();
		}
		
		if( cfg.exists("imgSources") )
		{
			libconfig::Setting &imgSrcSetting  = cfg.lookup("imgSources");
			assert( imgSrcSetting.getLength() == calibsSetting.getLength() );
			
			for( unsigned sc = 0; sc < imgSrcSetting.getLength(); ++sc )
			{
				std::stringstream iss;
				iss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) imgSrcSetting[sc];
				
				auto sp = CreateSource( iss.str() ); // doesn't matter that the source doesn't have the calibration, we have the calibration in the occSettings anyway.
			}
		}
		
		
		std::string s = (const char*)cfg.lookup("skelType");
		if( s.compare("open") == 0 || s.compare("openpose") == 0 )
		{
			data.skelType = SKEL_OPOSE;
		}
		if( s.compare("alpha") == 0 || s.compare("alphapose") == 0 )
		{
			data.skelType = SKEL_APOSE;
		}
		if( s.compare("dlc") == 0 || s.compare("deeplabcut") == 0 )
		{
			data.skelType = SKEL_DLCUT;
		}
		
		
		data.occSettings.minX = cfg.lookup("minX");
		data.occSettings.maxX = cfg.lookup("maxX");
		data.occSettings.minY = cfg.lookup("minY");
		data.occSettings.maxY = cfg.lookup("maxY");
// 		data.occSettings.minZ = cfg.lookup("minZ");  // not using because always z-up
// 		data.occSettings.maxZ = cfg.lookup("maxZ");
		
		std::string upDir = (const char*) cfg.lookup("upDir");
		if( upDir.compare("x") == 0 )
		{
			data.occSettings.upDir = OccupancyMap::UP_X;
		}
		else if( upDir.compare("y") == 0 )
		{
			data.occSettings.upDir = OccupancyMap::UP_Y;
		}
		else if( upDir.compare("z") == 0 )
		{
			data.occSettings.upDir = OccupancyMap::UP_Z;
		}
		
		data.minConf           = cfg.lookup("minJointConfidence");
		data.distanceThreshold = cfg.lookup("singleJointDistanceThreshold");
		data.minInliers        = cfg.lookup("singleJointMinInliers");
		
		std::string lrdstr = (const char*) cfg.lookup("leftRightDecision");
		if( lrdstr.compare("xplanepos") == 0 )
		{
			data.lrd = LRD_XPLANE_POS;
		}
		else if( lrdstr.compare("yplanepos") == 0 )
		{
			data.lrd = LRD_YPLANE_POS;
		}
		else if( lrdstr.compare("zplanepos") == 0 )
		{
			data.lrd = LRD_ZPLANE_POS;
		}
		if( lrdstr.compare("xplaneneg") == 0 )
		{
			data.lrd = LRD_XPLANE_NEG;
		}
		else if( lrdstr.compare("yplaneneg") == 0 )
		{
			data.lrd = LRD_YPLANE_NEG;
		}
		else if( lrdstr.compare("zplaneneg") == 0 )
		{
			data.lrd = LRD_ZPLANE_NEG;
		}
		else if( lrdstr.compare("votes") == 0 )
		{
			data.lrd = LRD_VOTE;
		}
		
		
		data.firstFrame = 0;
		if( cfg.exists("firstFrame") )
		{
			data.firstFrame = cfg.lookup("firstFrame");
		}
		
		data.visualise = false;
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
		}
		
		std::stringstream ass;
		ass << data.dataRoot << "/" << data.testRoot << "/" << (const char*) cfg.lookup("assocFile");
		data.assocFile = ass.str();
		
		std::stringstream oss;
		oss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) cfg.lookup("reconDir");
		data.reconDir = oss.str();
		
		if( cfg.exists("saveC3D") && (bool)cfg.lookup("saveC3D"))
		{
			data.writeToC3D = true;
			
			if( cfg.exists("C3DOffsetFile") )
			{
				std::string s = data.dataRoot + data.testRoot + (const char*)cfg.lookup("C3DOffsetFile");
				
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
}


void LoadPoses( SData &data )
{
	data.pcPoses.resize( data.poseSources.size() );
	
	for( unsigned sc = 0; sc < data.poseSources.size(); ++sc )
	{
		// type of skeleton affects how we load data :(
		switch( data.skelType )
		{
			case SKEL_OPOSE:
			case SKEL_APOSE:
				ReadPoseDirJSON( data.skelType, data.poseSources[sc], data.pcPoses[sc] );
				break;
			
			case SKEL_DLCUT:
				ReadDLC_CSV( data.poseSources[sc], data.pcPoses[sc] );
				break;
		}
	}
}



int main( int argc, char* argv[] )
{
	if( argc != 2 )
	{
		cout << argv[0] << " is a tool to load sparse pose detections" << endl;
		cout << "such as from OpenPose, AlphaPose, etc... and perform" << endl;
		cout << "cross-camera person association and tracking."        << endl;
		cout << "This tool does not do 3D pose reconstruction, but is" << endl;
		cout << "a precursor of the fuseSparsePoses tool, which does." << endl;
		cout << endl;
		cout << "Usage:" << endl;
		cout << argv[0] << " <config file> " << endl;
		exit(0);
	}
	
	//
	// Parse the config file
	//
	SData data;
	ParseConfig( argv[1], data );
	
	
	//
	// Load the pose data.
	//
	LoadPoses( data );
	
	
	//
	// Load up the tracking / cross-camera association data.
	//
	std::vector< std::map< int, std::vector< std::pair<int,int> > > > tracks; // [track][frame][ cam, person ]
	std::ifstream infi( data.assocFile );
	int numTracks;
	infi >> numTracks;
	if( !infi )
	{
		cout << "Could not load track / associations file : " << data.assocFile << endl;
		exit(0);
	}
	
	tracks.resize(numTracks);
	for(unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		int numFrames;
		infi >> numFrames;
		for( unsigned fc = 0; fc < numFrames; ++fc )
		{
			int frameNo;
			infi >> frameNo;
			
			int numAssocs;
			infi >> numAssocs;
			
			for( unsigned ac = 0; ac < numAssocs; ++ac )
			{
				int cam;
				int person;
				infi >> cam;
				infi >> person;
				
				tracks[tc][ frameNo ].push_back( std::pair<int,int>( cam, person ) );
			}
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//
	// So now we have to do the real work.
	// 
	std::vector< std::map< int, PersonPose3D > > tracksFused( tracks.size() );
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		std::map< int, PersonPose3D > fusedFrames;
		for( auto fi = tracks[tc].begin(); fi != tracks[tc].end(); ++fi )
		{
			cout << tc << " " << fi->first << endl;
			//
			// We can initialise a PersonPose3D with links to the per-cam data.
			//
			PersonPose3D pers3d;
			int frame = fi->first;
			for( unsigned ac = 0; ac < fi->second.size(); ++ac )
			{
				int cam = fi->second[ac].first;
				int pindx = fi->second[ac].second;
				pers3d.camPers[ cam ] = data.pcPoses[ cam ][ frame ][ pindx ];
			}
			
			
			//
			// And then we use the relevant fusion method to get the 3D pose.
			//
			ReconstructPerson( pers3d, data.skelType, data.occSettings.calibs, data.minConf, data.lrd, data.minInliers, data.distanceThreshold );
			
			fusedFrames[ frame ] = pers3d;
		}
		
		tracksFused[tc] = fusedFrames;
	}
	
	
	//
	// And now we can dump our output.
	//
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		SaveBody( tracksFused[tc], tc,  data );
	}
}


void SaveBody( std::map< int, PersonPose3D > &track, int trackNo, SData &data )
{
	std::stringstream tss;
	tss << data.reconDir << "/body-" << std::setw(2) << std::setfill('0') << trackNo << ".trk";
	
	std::stringstream css;
	css << data.reconDir << "/body-" << std::setw(2) << std::setfill('0') << trackNo << ".c3d";
	
	boost::filesystem::path p(data.reconDir);
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
	
	// basic text file output
	std::ofstream outfi( tss.str() );
	if( outfi )
		cout << "Could not open : " << tss.str() << " for output." << endl;
	
	outfi << data.testRoot << endl;
	
	// iterate over frames...
	for( auto fi = track.begin(); fi != track.end(); ++fi )
	{
		outfi << fi->first << "\t";
		for( auto ji = fi->second.joints.begin(); ji != fi->second.joints.end(); ++ji )
		{
			outfi << ji->first << " " << ji->second.transpose() << "\t";
		}
		outfi << endl;
	}
	
	outfi.close();
	
	//
	// Or better yet, output to a .c3d file. Which might mean we have a slight time offset to consider,
	// so that our .c3d file lines up with any marker-based mocap data we have.
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
		if( data.skelType == SKEL_OPOSE )
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
		else if( data.skelType == SKEL_APOSE )
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
		else if( data.skelType == SKEL_DLCUT )
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
		for( int fc2 = 0; fc2 < track.rbegin()->first; ++fc2 )
		{
			if( fc2 + data.mocapOffset < 0 )
			{
				cout << fc << " " << fc2 << " skip ( " << fc2 + data.mocapOffset << " ) " << endl;
				continue;
			}
			
			auto fi = track.find( fc2 );
			if( fi == track.end() )
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
		
		c3d.write( css.str() );
		
#else
		cout << "you asked to write to c3d, but I was compiled without ezc3d support :( " << endl;
#endif
	}
}
