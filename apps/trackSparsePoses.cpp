#include "recon/poseFusion/poseFusion.h"

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
	
	// [cam][frame][person]
	std::vector< std::map< int, PersonPose > > pcPoses;
	
	skeleton_t skelType;
	
	int firstFrame;
}


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
		
		libconfig::Setting &imgSrcSetting  = cfg.lookup("imgSources");
		libconfig::Setting &calibsSetting  = cfg.lookup("calibFiles");
		libconfig::Setting &poseSrcSetting = cfg.lookup("poseSources");
		
		assert( poseSrcSetting.getLength() == calibs.getLength() );
		
		data.maskSources.resize( maskDirs.getLength() );
		data.occSettings.calibs.resize( maskDirs.getLength() );
		for( unsigned sc = 0; sc < maskDirs.getLength(); ++sc )
		{
			std::stringstream mss, css;
			mss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) maskDirs[sc];
			css << data.dataRoot << "/" << data.testRoot << "/" << (const char*) calibs[sc];
			
			cout << css.str() << endl;
			cout << mss.str() << endl;
			data.maskSources[sc].reset( new ImageDirectory(mss.str(), css.str()) );
			
			data.occSettings.calibs[sc] = data.maskSources[sc]->GetCalibration();
		}
		
		
		data.occSettings.minX = cfg.lookup("minX");
		data.occSettings.maxX = cfg.lookup("maxX");
		data.occSettings.minY = cfg.lookup("minY");
		data.occSettings.maxY = cfg.lookup("maxY");
		data.occSettings.minZ = cfg.lookup("minZ");
		data.occSettings.maxZ = cfg.lookup("maxZ");
		
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
	ParseConfig( argv[1] );
	
	
	//
	// 
	//
	
}
