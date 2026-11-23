#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "renderer2/showImage.h"

#include "commonConfig/commonConfig.h"

#include "recon/occupancy.h"
#include "tracking/occupancyTracker.h"

#include "imgio/loadsave.h"
#include "imgio/imagesource.h"


struct SData
{
	std::string dataRoot;
	std::string testRoot;
	
	OccupancyMap::SOccMapSettings occSettings;
	OccupancyTracker::SOccTrackSettings trackSettings;
	
	std::vector< std::string > calibFiles;
	std::vector< std::shared_ptr<ImageSource> > maskSources;
	
	int firstFrame;
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
		
		libconfig::Setting &maskDirs = cfg.lookup("maskDirs");
		libconfig::Setting &calibs   = cfg.lookup("calibFiles");
		
		assert( maskDirs.getLength() == calibs.getLength() );
		
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
		
		data.trackSettings.numTracksGuide = cfg.lookup("numTracksGuide");
		
		
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


int main(int argc, char *argv[] )
{
	//
	// Basic stuff...
	//
	
	if( argc != 2 )
	{
		cout << "Test of occupancy map and occupancy tracker" << endl;
		cout << "Must provide source images etc.            " << endl;
		cout << "  - note, test assumes occupancy map is    " << endl;
		cout << "    made from binary foreground/background " << endl;
		cout << "    masks.                                 " << endl;
		cout << "                                           " << endl;
		cout << argv[0] << " <config file> "                  << endl;
		cout << endl;
		
		return -1;
	}
	
	
	SData data;
	ParseConfig( argv[1], data );
	
	//
	// Create our occupancy mapper and our occupancy tracker
	//
	OccupancyMap     OM( data.occSettings );
	
	std::vector< cv::Mat > visMaps;
	OM.GetLineVisibility(visMaps);
	data.trackSettings.visMap = visMaps[0];
	data.trackSettings.numCameras = data.maskSources.size();
	OccupancyTracker OT( data.trackSettings );
	
	cout << "Jumping to frame: " << data.firstFrame << endl;
	for( unsigned mc = 0; mc < data.maskSources.size(); ++mc )
	{
		data.maskSources[mc]->JumpToFrame( data.firstFrame );
	}
	
	//
	// Create a debug renderer.
	//
	int mapRows = OM.GetMapRows();
	int mapCols = OM.GetMapCols();
	
	CommonConfig ccfg;
	int renW, renH;
	float rat = (float)mapRows / (float)mapCols;
	
	renW = ccfg.maxSingleWindowWidth;
	renH = rat * renW;
	if( renH > ccfg.maxSingleWindowHeight )
	{
		renH = ccfg.maxSingleWindowHeight;
		renW = renH / rat;
	}
	
	
	std::shared_ptr< Rendering::BasicPauseRenderer > ren;
	Rendering::RendererFactory::Create( ren, renW, renH, "tst" );
	ren->Get2dBgCamera()->SetOrthoProjection( 0, mapCols, 0, mapRows, -10, 10 );
	
	//
	// Loop over the frames generating occupancy maps and feeding data 
	// to the tracker.
	//
	bool done = false;
	std::vector< cv::Mat > masks( data.maskSources.size() );
	cv::Mat visOcc;
	bool paused, advance;
	paused = advance = false;
	while( !done )
	{
		int fc = data.maskSources[0]->GetCurrentFrameID();
		
		cout << "----" << fc << "----" << endl;
		
		for( unsigned mc = 0; mc < data.maskSources.size(); ++mc )
		{
			masks[mc] = data.maskSources[mc]->GetCurrent();
		}
		
		cout << "occ..." << endl;
		std::vector< cv::Mat > occ;
		OM.OccupancyFromSegmentation( masks, OccupancyMap::PROJ_LINE, occ );
		
		
		cout << "add frame..." << endl;
		visOcc = OT.AddFrame( fc, occ[0] );
		
		visOcc = cv::Mat( visOcc.rows, visOcc.cols, CV_32FC3, cv::Scalar(0,0,0) );
		for( unsigned rc = 0; rc < visOcc.rows; ++rc )
		{
			for( unsigned cc = 0; cc < visOcc.cols; ++cc )
			{
				cv::Vec3f &vp = visOcc.at< cv::Vec3f >(rc,cc);
				float &v = visMaps[0].at< float >(rc,cc);
				float &o = occ[0].at<float>(rc,cc);
				
				vp[0] = v / (float)data.maskSources.size();
				vp[1] = o / v;
				
			}
		}
		
		ren->SetBGImage(visOcc);
		
		ren->Step(paused, advance);
		
		cout << "advance..." << endl;
		for( unsigned mc = 0; mc < data.maskSources.size(); ++mc )
		{
			done = done || !data.maskSources[mc]->Advance();
		}
	}
	
	cout << "get tracks..." << endl;
	
	//
	// Get our final tracks and visualise them.
	//
	std::vector< OccupancyTracker::STrack > tracks;
	//OT.GetTracks(tracks, ren, visOcc);
	
	OT.GetTracks(tracks);
	cout << "tracks: " << tracks.size() << endl;
	while(1)
	{
		for( unsigned tc = 0; tc < tracks.size(); ++tc )
		{
			cout << tc << endl;
			auto fi00 = tracks[tc].framePeaks.begin();
			while( fi00 != tracks[tc].framePeaks.end() )
			{
				OccupancyTracker::SPeak &peak = fi00->second;
				
				// start by drawing the gaussian of this frame peak.
				visOcc = cv::Mat( visOcc.rows, visOcc.cols, CV_32FC3, cv::Scalar(0,0,0) );
				
				#pragma omp parallel for
				for( unsigned rc = 0; rc < visOcc.rows; ++rc )
				{
					for( unsigned cc = 0; cc < visOcc.cols; ++cc )
					{
						cv::Vec3f &p = visOcc.at< cv::Vec3f >(rc,cc);
						float &v = p[2];
						
						// given a 2D Gaussian with mean and cov what is the value at (cc,rc)?
						hVec2D d; d << cc,rc,1.0f;
						d = d - peak.mean;
						float top = exp( -0.5 * d.head(2).transpose() * peak.cov.inverse() * d.head(2) );
						//float bot = sqrt( (2*3.1415*peak.cov).norm() ); // don't care about this for vis.
						
						p[1] = std::max( p[1], top * peak.confidence ) ;
					}
				}
				
				
				// then over the top of that, draw the whole track as lines.
				auto fi0 = tracks[tc].framePeaks.begin();
				auto fi1 = tracks[tc].framePeaks.begin();
				fi1++;
				while( fi1 != tracks[tc].framePeaks.end() )
				{
					cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
					cv::Point p1( fi1->second.mean(0), fi1->second.mean(1) );
					
					cv::line( visOcc, p0, p1, cv::Scalar(1,0,0), 2 );
					++fi0;
					++fi1;
				}
				
				ren->SetBGImage(visOcc);
				ren->Step(paused, advance);
				
				
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				++fi00;
			}
			
		}
	}
	
}

