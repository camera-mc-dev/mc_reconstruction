#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"

#include <iostream>
using std::cout;
using std::endl;


//
// The job of this program is to load up pose detections from multiple
// camera views, associate the people between different views, and output
// the tracks and associations. It is strongly similar to trackSparsePoses,
// but uses a different approach.
//  - sparse poses      : Each pose detection is converted to a bounding box
//  - bb detections     : used directly 
//  - segmentation masks: used as is, or converted to bounding boxes
//
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

enum segMask_t {SEG_DPOSE_ME};

struct SData
{
	std::string dataRoot;
	std::string testRoot;
	
	OccupancyMap::SOccMapSettings occSettings;
	OccupancyTracker::SOccTrackSettings trackSettings;
	
	std::vector< std::string > calibFiles;
	std::vector< std::shared_ptr<ImageSource> > imgSources;
	
	std::vector< std::string > segSourceStrs;
	std::vector< std::shared_ptr<ImageSource> > segSources;
	segMask_t segType;
	bool seg2Rect;
	
	std::vector< std::string > poseSources;
	skeleton_t skelType;
	
	// [cam][frame][person]
	std::vector< std::map< int, std::vector<PersonPose> > > pcPoses;
	
	
	int firstFrame;
	
	bool visualise;
	
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
		
		assert( 
		        cfg.exists("poseSources") || cfg.exists("segSources")
		      );
		
		//
		// Read calibs
		//
		data.occSettings.calibs.resize( calibsSetting.getLength() );
		for( unsigned sc = 0; sc < calibsSetting.getLength(); ++sc )
		{
			std::stringstream css;
			css << data.dataRoot << "/" << data.testRoot << "/" << (const char*) calibsSetting[sc];
			
			cout << css.str() << endl;
			
			data.occSettings.calibs[sc].Read( css.str() );
		}
		
		//
		// Make sure we don't have pose sources _and_ mask sources.
		// We could, in theory, mix them, but for now, we wont.
		//
		if( cfg.exists("poseSources") && cfg.exists("segSources") )
		{
			throw std::runtime_error("Can't mix pose sources and segmentation sources");
		}
		
		//
		// Read pose sources (if we have them )
		//
		if( cfg.exists("poseSources") )
		{
			libconfig::Setting &poseSrcSetting = cfg.lookup("poseSources");
			assert( poseSrcSetting.getLength() == calibsSetting.getLength() );
			
			data.poseSources.resize( poseSrcSetting.getLength() );
			for( unsigned sc = 0; sc < poseSrcSetting.getLength(); ++sc )
			{
				std::stringstream pss;
				pss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) poseSrcSetting[sc];
				data.poseSources[sc] = pss.str();
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
		}
		
		//
		// Read segmentation sources (if we have them);
		//
		if( cfg.exists("segSources") )
		{
			libconfig::Setting &segSrcSetting  = cfg.lookup("segSources");
			assert( segSrcSetting.getLength() == calibsSetting.getLength() );
			
			data.poseSources.resize( segSrcSetting.getLength() );
			for( unsigned sc = 0; sc < segSrcSetting.getLength(); ++sc )
			{
				std::stringstream sss;
				sss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) segSrcSetting[sc];
				data.segSourceStrs[sc] = sss.str();
			}
			
			
			std::string s = (const char*)cfg.lookup("segType");
			if( s.compare("dpose") == 0 || s.compare("densepose") == 0 )
			{
				data.segType = SEG_DPOSE_ME;
			}
			
			data.seg2Rect = cfg.lookup("seg2Rect");
		}
		
		//
		// Read image sources (if we have them)
		//
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
	if( data.poseSources.size() > 0 )
	{
		LoadPoses( data );
	}
	
	//
	// Open up the segmentation sources.
	//
	if( data.segSourceStrs.size() > 0 )
	{
		data.segSources.resize( data.segSourceStrs.size() );
		if( data.segType == SEG_DPOSE_ME )
		{
			// My DensePose output consists of two 
			// image directories, one for fg/bg/personID
			// and the other one for bodypart/U/V
			//
			// We will assume the source string points to the root 
			// of those two directories and we find the fgbg dir.
			std::stringstream ss;
			for( unsigned sc = 0; sc < data.segSourceStrs.size(); ++sc )
			{
				ss << data.segSourceStrs[sc] << "/fgbg/";
				auto sp = CreateSource( ss.str() );
				data.segSources[sc] = sp.source;
			}
		}
	}
	
	
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
	int minFrame = 0;
	int maxFrame = 0;
	
	if( data.poseSources.size() > 0 )
	{
		for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
		{
			if( data.pcPoses[sc].size() > 0 )
			{
				auto i = data.pcPoses[sc].rbegin();
				maxFrame = std::max( i->first, maxFrame );
			}
		}
		
		minFrame = maxFrame;
		for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
		{
			if( data.pcPoses[sc].size() > 0 )
			{
				auto i = data.pcPoses[sc].begin();
				minFrame = std::min( i->first, minFrame );
			}
		}
	}
	else
	{
		minFrame = 0;
		maxFrame = 999999999;
		for( unsigned sc = 0; sc < data.segSources.size(); ++sc )
		{
			maxFrame = std::min( data.segSources[sc]->GetNumImages(), maxFrame );
		}
	}
	
	cout << minFrame << " -> " << maxFrame << endl;
	
	int mapRows = OM.GetMapRows();
	int mapCols = OM.GetMapCols();
	
	
	std::shared_ptr< Rendering::BasicPauseRenderer > ren;
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
		
		
		Rendering::RendererFactory::Create( ren, renW, renH, "tst" );
		ren->Get2dBgCamera()->SetOrthoProjection( 0, mapCols, 0, mapRows, -10, 10 );
	}
	
	std::vector< cv::Mat > occ;
	for( unsigned fc = minFrame; fc < maxFrame; ++fc )
	{
		cout << fc << endl;
		
		//
		// Now we get down to buisness.
		//
		
		//
		// Are we doing this with bounding boxes or actual segmentations?
		//
		if( data.segSources.size() > 0 && !data.seg2Rect )
		{
			//
			// Use the segmentations directly for occupancy... HOWEVER...
			// for the sake of the final association procedure, we'll 
			// keep some bounding boxes of each person detection
			//
			std::vector< std::vector< cv::Rect > > bboxes( data.segSources.size() );
			if( data.pcPoses.size() != data.segSources.size() )
				data.pcPoses.resize( data.segSources.size() );
			std::map< int, std::pair< hVec2D, hVec2D > > minMax;
			
			// first off, we need binary segmentation masks
			// well, I guess they don't have to be binary, but, anyway...
			std::vector< cv::Mat > masks( data.segSources.size() );
			for( unsigned sc = 0; sc < data.segSources.size(); ++sc )
			{
				if( data.segType == SEG_DPOSE_ME )
				{
					cv::Mat fgbg = data.segSources[sc]->GetCurrent();
					masks[sc] = cv::Mat( fgbg.rows, fgbg.cols, CV_32FC1, cv::Scalar(0) );
					for( unsigned rc = 0; rc < fgbg.rows; ++rc )
					{
						for( unsigned cc = 0; cc < fgbg.cols; ++cc )
						{
							cv::Vec3f &ip = fgbg.at< cv::Vec3f >(rc,cc);
							masks[sc].at<float>(rc,cc) = std::max(0.0f, ip[1] - ip[0]);
							
							int personID = ip[2];
							if( personID > 0 )
							{
								auto i = minMax.find( personID );
								if( i != minMax.end() )
								{
									i->second.first(0) = std::min( i->second.first(0), (float)cc );
									i->second.first(1) = std::min( i->second.first(1), (float)rc );
									
									i->second.second(0) = std::max( i->second.second(0), (float)cc );
									i->second.second(1) = std::max( i->second.second(1), (float)rc );
								}
								else
								{
									minMax[ personID ].first << cc,rc,1.0f;
									minMax[ personID ].second << cc,rc,1.0f;
								}
							}
						}
					}
					
					for( auto pi = minMax.begin(); pi != minMax.end(); ++pi )
					{
						PersonPose pp;
						pp.representativeBB.x = pi->second.first(0);
						pp.representativeBB.y = pi->second.first(1);
						
						pp.representativeBB.width  = pi->second.second(0) - pi->second.first(0);
						pp.representativeBB.height = pi->second.second(1) - pi->second.first(1);
						
						pp.personID = pi->first;
						
						data.pcPoses[ sc ][ fc ].push_back( pp );
						
						bboxes[sc].push_back( pp.representativeBB );
					}
				}
			}
			
			//
			// Then we just compute occupancy as we always have.
			//
			OM.OccupancyFromSegmentation( masks, OccupancyMap::PROJ_LINE, occ );
		}
		else if( data.segSources.size() > 0 && data.seg2Rect )
		{
			//
			// Convert segmentations to bboxes for occupancy
			// You might ask - why would we do this? The computation might 
			// we a little quicker, maybe, but we lose definition on the 
			// shape of the mask. But, maybe we don't care about that, and 
			// mostly just care about presence, and in that case, we can
			// gain flexibility here.
			//
			std::vector< std::vector< cv::Rect > > bboxes( data.segSources.size() );
			
			// process the segmentation masks to get bounding boxes instead.
			// we'll also make pcPoses from this to keep the bboxes.
			if( data.pcPoses.size() != data.segSources.size() )
				data.pcPoses.resize( data.segSources.size() );
			
			std::map< int, std::pair< hVec2D, hVec2D > > minMax;
			for( unsigned sc = 0; sc < data.segSources.size(); ++sc )
			{
				if( data.segType == SEG_DPOSE_ME )
				{
					cv::Mat fgbg = data.segSources[sc]->GetCurrent();
					for( unsigned rc = 0; rc < fgbg.rows; ++rc )
					{
						for( unsigned cc = 0; cc < fgbg.cols; ++cc )
						{
							cv::Vec3f &ip = fgbg.at< cv::Vec3f >(rc,cc);
							int personID = ip[2];
							if( personID > 0 )
							{
								auto i = minMax.find( personID );
								if( i != minMax.end() )
								{
									i->second.first(0) = std::min( i->second.first(0), (float)cc );
									i->second.first(1) = std::min( i->second.first(1), (float)rc );
									
									i->second.second(0) = std::max( i->second.second(0), (float)cc );
									i->second.second(1) = std::max( i->second.second(1), (float)rc );
								}
								else
								{
									minMax[ personID ].first << cc,rc,1.0f;
									minMax[ personID ].second << cc,rc,1.0f;
								}
							}
						}
					}
					
					for( auto pi = minMax.begin(); pi != minMax.end(); ++pi )
					{
						PersonPose pp;
						pp.representativeBB.x = pi->second.first(0);
						pp.representativeBB.y = pi->second.first(1);
						
						pp.representativeBB.width  = pi->second.second(0) - pi->second.first(0);
						pp.representativeBB.height = pi->second.second(1) - pi->second.first(1);
						
						pp.personID = pi->first;
						
						data.pcPoses[ sc ][ fc ].push_back( pp );
						
						bboxes[sc].push_back( pp.representativeBB );
					}
				}
			}
				
			//
			// Then we just compute occupancy as we always have.
			//
			OM.OccupancyFromBBoxes( bboxes, occ );
			
		}
		else if( data.poseSources.size() > 0 )
		{
			//
			// Use bboxes for occupancy
			//
			std::vector< std::vector< cv::Rect > > bboxes( data.poseSources.size() );
			
			for( unsigned sc = 0; sc < data.pcPoses.size(); ++sc )
			{
				auto pf = data.pcPoses[sc].find( fc );
				if( pf != data.pcPoses[sc].end() )
				{
					for( unsigned pc = 0; pc < pf->second.size(); ++pc )
					{
						bboxes[sc].push_back( pf->second[pc].representativeBB );
					}
				}
			}
			
			//
			// Then we just compute occupancy as we always have.
			//
			OM.OccupancyFromBBoxes( bboxes, occ );
		}
		
		cv::Mat visOcc;
		visOcc = OT.AddFrame( fc, occ[0] );
		
		cv::minMaxIdx( visOcc, &m, &M );
		
		if( data.visualise )
		{
			ren->SetBGImage( visOcc );
			ren->StepEventLoop();
		}
		
		
		if( data.segSources.size() > 0 )
		{
			for( unsigned sc = 0; sc < data.segSources.size(); ++sc )
			{
				data.segSources[sc]->Advance();
			}
		}
	}
	
	
	//
	// Get our tracks...
	//
	std::vector< OccupancyTracker::STrack > tracks;
	OT.GetTracks(tracks);
	cout << "tracks: " << tracks.size() << endl;
	
	
	if( data.visualise )
	{
		for( unsigned tc = 0; tc < tracks.size(); ++tc )
		{
			cout << tc << endl;
			auto fi00 = tracks[tc].framePeaks.begin();
			while( fi00 != tracks[tc].framePeaks.end() )
			{
				OccupancyTracker::SPeak &peak = fi00->second;
				
				// start by drawing the gaussian of this frame peak.
				cv::Mat visOcc = cv::Mat( mapRows, mapCols, CV_32FC3, cv::Scalar(0,0,0) );
				
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
				ren->StepEventLoop();
				
				
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				++fi00;
			}
			
		}
	}
	
	
	
	
	//
	// We now need to do the actual cross-camera person associations,
	// and in the process, we need to create our output file.
	//
	// The way that we associate a track to a detection is basically to 
	// project the track peak into the views and compare that with 
	// the per-detection bounding boxes that we created.
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
	// As simple as I can for now.
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
			// Intersect detection bounding box with the cell's bounding box.
			//
			cv::Rect &dbb = detections[dc].representativeBB;
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
