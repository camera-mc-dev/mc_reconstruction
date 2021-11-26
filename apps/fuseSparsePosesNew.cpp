#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"

#include <iostream>
using std::cout;
using std::endl;


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
	
	int firstFrame;
	
	bool visualise;
	
	std::string assocFile;
};

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
		
		data.occSettings.cellSize = cfg.lookup("cellSize");
		
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
		
		
		OccupancyMap::SObsPlane obsPlane;
		obsPlane.low  =  cfg.lookup("planeLow");
		obsPlane.high = cfg.lookup("planeHigh");
		data.occSettings.obsPlanes.push_back( obsPlane );
		
		data.occSettings.cellPadding = cfg.lookup("cellPadding");
		
		
		data.trackSettings.minVisibility = cfg.lookup("minVisibility");
		data.trackSettings.useVisibility = cfg.lookup("useVisibility");
		data.trackSettings.detectionThreshold = cfg.lookup("detectionThreshold");
		
		data.trackSettings.distanceThreshold = 12.0f;
		data.trackSettings.numNearPeaks      = 50;
		
		
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
		
		std::stringstream oss;
		oss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) cfg.lookup("assocFile");
		data.assocFile = oss.str();
		
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
			ReconstructPerson( pers3d, data.occSettings.calibs );
			
			fusedFrames[ frame ] = pers3d;
		}
		
		tracksFused[tc] = fusedFrames;
	}
	
	
	//
	// And now we can dump our output.
	//
}
