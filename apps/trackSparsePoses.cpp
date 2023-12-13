#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"
#include "renderer2/renWrapper.h"

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
	
	poseSource_t poseDataType;
	
	int firstFrame;
	bool occupancyFromPoints;
	bool visualise;
	bool renderHeadless;
	std::string renderTarget;
	
	std::string outfile;
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
			
			if( !data.occSettings.calibs[sc].Read( css.str() ) )
			{
				throw std::runtime_error( "Could not open calib file: " + css.str() );
			}
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
		
		
		std::string s = (const char*)cfg.lookup("poseDataType");
		if( s.compare("jsonDir") == 0 )  // directory of JSON files
		{
			data.poseDataType = POSE_JSON_DIR;
		}
		if( s.compare("dlccsv") == 0 )
		{
			data.poseDataType = POSE_DLC_CSV;
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
		
		data.occupancyFromPoints = cfg.lookup("occupancyFromPoints");
		
		data.visualise = false;
		data.renderHeadless = false;
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
			
			
			if( cfg.exists("renderHeadless") )
			{
				data.renderHeadless = cfg.lookup("renderHeadless");
				if( data.renderHeadless )
					data.renderTarget   = (const char*) cfg.lookup("renderTarget" );
			}
		}
		
		std::stringstream oss;
		oss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) cfg.lookup("assocFile");
		data.outfile = oss.str();
		
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
	
	#pragma omp parallel for
	for( unsigned sc = 0; sc < data.poseSources.size(); ++sc )
	{
		// type of skeleton affects how we load data :(
		switch( data.poseDataType )
		{
			case POSE_JSON_DIR:
				cout << "reading poses from directory of .json files: " << data.poseSources[sc] << endl;
				ReadPoseDirJSON( data.poseSources[sc], data.pcPoses[sc] );
				break;
			
			case POSE_DLC_CSV:
				cout << "reading poses from deep lab cut .csv file: " << data.poseSources[sc] << endl;
				ReadDLC_CSV( data.poseSources[sc], data.pcPoses[sc] );
				break;
		}
	}
}


int GetPersonForTrack( int frameNo, int view, cv::Rect cellBB, SData &data );


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
	// Create the occupancy map and its tracker.
	//
	OccupancyMap     OM( data.occSettings );
	
	std::vector< cv::Mat > visMaps;
	OM.GetBBVisibility(visMaps);
	data.trackSettings.visMap = visMaps[0];
	data.trackSettings.numCameras = data.occSettings.calibs.size();
	OccupancyTracker OT( data.trackSettings );
	
	
	
	//
	// for all the frames that we have data, compute occupancy
	//
	int maxFrame = 0;
	for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
	{
		if( data.pcPoses[sc].size() > 0 )
		{
			auto i = data.pcPoses[sc].rbegin();
			maxFrame = std::max( i->first, maxFrame );
		}
	}
	
	int minFrame = maxFrame;
	for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
	{
		if( data.pcPoses[sc].size() > 0 )
		{
			auto i = data.pcPoses[sc].begin();
			minFrame = std::min( i->first, minFrame );
		}
	}
	
	cout << "Running detection on frames " << minFrame << " -> " << maxFrame << endl;
	
	int mapRows = OM.GetMapRows();
	int mapCols = OM.GetMapCols();
	
	typedef RenWrapper< Rendering::BasicPauseRenderer, Rendering::BasicHeadlessRenderer> rw_t;
	std::shared_ptr< rw_t > renWrapper;
	
	if( data.visualise )
	{
		float rat = (float)mapRows / (float)mapCols;
		
		
		CommonConfig ccfg;
		
		int renW = ccfg.maxSingleWindowWidth;
		int renH = rat * renW;
		
		if( renH > ccfg.maxSingleWindowHeight)
		{
			renH = ccfg.maxSingleWindowHeight;
			renW = renH / rat;
		}
		
		renWrapper.reset( new rw_t( data.renderHeadless, renW, renH, "tst" ) );
		
		renWrapper->Get2dBgCamera()->SetOrthoProjection( 0, mapCols, 0, mapRows, -10, 10 );
	}
	
	// If we're doing headless rendering, where are we writing the 
	// visuals to? We'll just write to directories for now.
	std::string occRenDir;
	if( data.renderHeadless )
	{
		std::stringstream ss; ss << data.renderTarget << "/occupancy/";
		occRenDir = ss.str();
		boost::filesystem::path p( occRenDir );
		if( boost::filesystem::exists(p) && !boost::filesystem::is_directory(p))
		{
			throw( std::runtime_error("occupancy render target exists but is not a directory") );
		}
		else if( !boost::filesystem::exists(p) )
		{
			boost::filesystem::create_directories(p);
		}
	}
	
	
	bool paused = false;
	std::vector< cv::Mat > occ;
	for( unsigned fc = minFrame; fc < maxFrame; ++fc )
	{
		cout << fc << endl;
		
		
		//
		// First version of this will use the representative point 
		// of each detection.
		//
		if( data.occupancyFromPoints )
		{
			std::vector< std::vector< hVec2D > > points(data.pcPoses.size());
			for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
			{
				auto i = data.pcPoses[sc].find(fc);
				if( i != data.pcPoses[sc].end() )
				{
					for( unsigned pc = 0; pc < i->second.size(); ++pc )
					{
						points[sc].push_back( i->second[pc].representativePoint );
					}
				}
			}
			
			
			OM.OccupancyFromPoints( points, occ );
		}
		else
		{
			//
			// Occupancy from bboxes makes more sense probably, though arms flailing out wide 
			// could be a complication...
			//
			std::vector< std::vector< cv::Rect > > bboxes( data.pcPoses.size() );
			for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
			{
				auto i = data.pcPoses[sc].find(fc);
				if( i != data.pcPoses[sc].end() )
				{
					for( unsigned pc = 0; pc < i->second.size(); ++pc )
					{
						bboxes[sc].push_back( i->second[pc].representativeBB );
					}
				}
			}
			
			
			OM.OccupancyFromBBoxes( bboxes, occ );
		}
		cv::Mat visOcc;
		visOcc = OT.AddFrame( fc, occ[0] );
		
		if( data.visualise )
		{
			renWrapper->SetBGImage( visOcc );
			
			if( data.renderHeadless )
			{
				renWrapper->StepEventLoop();
				cv::Mat grab = renWrapper->Capture();
				std::stringstream ss;
				ss << occRenDir << "/" << std::setw(6) << std::setfill('0') << fc << ".jpg";
				SaveImage( grab, ss.str() );
			}
			else
			{
				bool advance = false;
				renWrapper->ren->Step(paused, advance);
				while( paused && !advance )
				{
					renWrapper->ren->Step(paused, advance);
				}
			}
		}
		
	}
	
	
	//
	// Get our tracks...
	//
	std::vector< OccupancyTracker::STrack > tracks;
#ifdef OCCTRACK_DEBUG
	cv::Mat debug( mapRows, mapCols, CV_32FC3, cv::Scalar(0,0,0) );
	OT.GetTracks(tracks, ren, debug );
#else
	OT.GetTracks(tracks);
#endif
	cout << "tracks: " << tracks.size() << endl;
	
	
	if( data.visualise )
	{
		cout << "Rendering tracks: " << endl;
		paused = false;
		for( unsigned tc = 0; tc < tracks.size(); ++tc )
		{
			std::string trkRenDir;
			if( data.renderHeadless )
			{
				std::stringstream ss;
				ss << data.renderTarget << "/trk" << std::setw(3) << std::setfill('0') << tc << "/";
				trkRenDir = ss.str();
				boost::filesystem::path p( trkRenDir );
				if( boost::filesystem::exists(p) && !boost::filesystem::is_directory(p))
				{
					throw( std::runtime_error("track render target exists but is not a directory") );
				}
				else if( !boost::filesystem::exists(p) )
				{
					boost::filesystem::create_directories(p);
				}
			}
			
			
			cout << tc << endl;
			auto fi00 = tracks[tc].framePeaks.begin();
			for(unsigned fc = minFrame; fc < maxFrame; ++fc )
			{
				cv::Mat visOcc = cv::Mat( mapRows, mapCols, CV_32FC3, cv::Scalar(0,0,0) );
					
				auto fi00 = tracks[tc].framePeaks.find( fc );
				if( fi00 != tracks[tc].framePeaks.end() )
				{
					OccupancyTracker::SPeak &peak = fi00->second;
					
					// start by drawing the gaussian of this frame peak.
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
				
				renWrapper->SetBGImage( visOcc );
			
				if( data.renderHeadless )
				{
					renWrapper->StepEventLoop();
					cv::Mat grab = renWrapper->Capture();
					std::stringstream ss;
					ss << trkRenDir << "/" << std::setw(6) << std::setfill('0') << fc << ".jpg";
					SaveImage( grab, ss.str() );
				}
				else
				{
					bool advance = false;
					renWrapper->ren->Step(paused, advance);
					while( paused && !advance )
					{
						renWrapper->ren->Step(paused, advance);
					}
				}
				
				
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				++fi00;
			}
			
		}
	}
	
	
	
	//
	// The final thing we need to do is create our output file, and 
	// actually do the cross-camera association (because so far, all
	// we've done is infer the location of things implied by the detections)
	//
	std::ofstream outfi(data.outfile);
	outfi << tracks.size() << endl;
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		outfi << tracks[tc].framePeaks.size() << endl;
		for( auto fi = tracks[tc].framePeaks.begin(); fi != tracks[tc].framePeaks.end(); ++fi )
		{
			// point on the map
			hVec2D mapPoint = fi->second.mean;
			
			//
			// go through each view and find the best person, if there is one.
			//
			std::vector< std::pair<int,int> > assocs;
			for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
			{
				//
				// cell as a bounding box in this view...
				//
				cv::Rect bb = OM.GetCellBBox(0, mapPoint(1), mapPoint(0), sc );
				
				int assoc = GetPersonForTrack( fi->first, sc, bb, data );
				if( assoc >= 0 )
				{
					assocs.push_back( std::pair<int,int>(sc,assoc) );
				}
				
			}
			
			outfi << fi->first << " " << assocs.size() << " ";
			for( unsigned sc = 0; sc < assocs.size(); ++sc )
			{
				outfi << assocs[sc].first << " " << assocs[sc].second << " ";
			}
			outfi << endl;
		}
	}
	
}


int GetPersonForTrack( int frameNo, int view, cv::Rect cellBB, SData &data )
{
	//
	// I don't really like this approach, but if it works, it works.
	//
	
	// get the frame...
	auto fi = data.pcPoses[view].find( frameNo );
	
	if( fi != data.pcPoses[view].end() && fi->second.size() > 0 )
	{
		std::vector< PersonPose > &detections = fi->second;
		
		float bestVal = 0.0f;
		int bestDet = -1;
		for( unsigned dc = 0; dc < detections.size(); ++dc )
		{
			//
			// Get a bounding box for all valid points of this detection.
			//
			std::vector< hVec2D > &jts = detections[dc].joints;
			std::vector< float > &cnfs = detections[dc].confidences;
			hVec2D m,M;
			bool first = true;
			for( unsigned jc = 0; jc < jts.size(); ++jc )
			{
				if( cnfs[jc] > 0.2 && jts[jc](2) > 0 )
				{
					if( !first )
					{
						m(0) = std::min( jts[jc](0), m(0) );
						m(1) = std::min( jts[jc](1), m(1) );
						M(0) = std::max( jts[jc](0), M(0) );
						M(1) = std::max( jts[jc](1), M(1) );
					}
					else
					{
						m = jts[jc];
						M = jts[jc];
						first = false;
					}
				}
			}
			
			if( first )
			{
				// not enough good points to get the bbox for the detection.
				// gets a bad score.
				continue;
			}
			
			
			//
			// Intersect that bounding box with the cell's bounding box.
			//
			cv::Rect dbb;
			dbb.x = m(0);
			dbb.y = m(1);
			dbb.width  = M(0) - m(0);
			dbb.height = M(1) - m(1);
			
			cv::Rect ibb;
			ibb = cellBB & dbb;
			
			if( ibb.width > 0 )
			{
				//
				// So at the very least, the detection overlaps the cell.
				// But how well?
				//
				// If we assume the cell is more or less person height,
				// then we demand that the detection be not massively larger 
				// or smaller than the cell height.
				//
				
				float hRat =        std::min( cellBB.height, dbb.height ) / 
				             (float)std::max( cellBB.height, dbb.height );
				
				
				if( hRat > bestVal )
				{
					bestVal = hRat;
					bestDet = dc;
				}
			}
		}
		
		return bestDet;
	}
	else
	{
		return -1;
	}
	
}
