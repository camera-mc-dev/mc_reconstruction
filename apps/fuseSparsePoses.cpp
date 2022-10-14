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
				
				data.imgSources.push_back( sp.source );
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





class FrameChangeRenderer : public Rendering::BasicRenderer
{
	friend class Rendering::RendererFactory;	
	// The constructor should be private or protected so that we are forced 
	// to use the factory...
protected:
	// the constructor creates the renderer with a window of the specified
 	// size, and with the specified title.
	FrameChangeRenderer(unsigned width, unsigned height, std::string title) : BasicRenderer(width,height,title) {}
public:
	bool Step(int &camChange, bool &paused, int &frameChange )
	{
		Render();
		
		win.setActive();
		sf::Event ev;
		while( win.pollEvent(ev) )
		{
			if( ev.type == sf::Event::Closed )
				return false;
			
			if (ev.type == sf::Event::KeyReleased)
			{
				if (ev.key.code == sf::Keyboard::Left )
				{
					camChange = 1;
				}
				if (ev.key.code == sf::Keyboard::Right )
				{
					camChange = -1;
				}
				if (ev.key.code == sf::Keyboard::Space )
				{
					paused = !paused;
				}
				if (ev.key.code == sf::Keyboard::Up )
				{
					frameChange = 1;
				}
				if (ev.key.code == sf::Keyboard::Down )
				{
					frameChange = -1;
				}
			}
		}
		return true;
	}
};







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
	
	
	
	
	std::shared_ptr< FrameChangeRenderer > ren;
	if( data.visualise )
	{
		assert( data.imgSources.size() == data.poseSources.size() );
		
		// I'm going to assume that the images from all the sources are the same size and shape.
		cv::Mat img = data.imgSources[0]->GetCurrent();
		
		// how big are the images?
		float ar = img.rows/ (float) img.cols;
		
		
		// create the renderer for display purposes.
		cout << "creating window" << endl;
		CommonConfig ccfg;
		float winW = ccfg.maxSingleWindowWidth;
		float winH = winW * ar;
		if( winH > ccfg.maxSingleWindowHeight )
		{
			winH = ccfg.maxSingleWindowHeight;
			winW = winH / ar;
		}
		if( data.visualise )
			Rendering::RendererFactory::Create( ren, winW, winH, "recon vis");
		
		ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
		ren->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	//
	// So now we have to do the real work.
	// 
	int camChange, frameAdvance;
	camChange = frameAdvance = 0;
	bool paused = true;
	int viewCam = 0;
	std::vector< std::map< int, PersonPose3D > > tracksFused( tracks.size() );
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		std::map< int, PersonPose3D > fusedFrames;
		for( auto fi = tracks[tc].begin(); fi != tracks[tc].end(); ++fi )
		{
			cout << tc << " " << fi->first << endl;
			if( fi->first < data.firstFrame )
				continue;
			//
			// We can initialise a PersonPose3D with links to the per-cam data.
			//
			PersonPose3D pers3d;
			int frame = fi->first;
			for( unsigned ac = 0; ac < fi->second.size(); ++ac )
			{
				int cam = fi->second[ac].first;
				int pindx = fi->second[ac].second;
				auto pi = data.pcPoses[cam].find(frame);
				if( pi != data.pcPoses[cam].end() )
				{
					if( pindx < pi->second.size() )
					{
						pers3d.camPers[ cam ] = pi->second[pindx];
					}
					else
					{
						cout << "Does that make any sense?" << endl;
					}
				}
			}
			
			
			//
			// And then we use the relevant fusion method to get the 3D pose.
			//
			ReconstructPerson( pers3d, data.skelType, data.occSettings.calibs, data.minConf, data.lrd, data.minInliers, data.distanceThreshold );
			
			fusedFrames[ frame ] = pers3d;
			
			if( data.visualise )
			{
				if( data.imgSources[viewCam]->GetCurrentFrameID() != fi->first )
					data.imgSources[viewCam]->JumpToFrame( fi->first );   // TODO: be more efficient
				cv::Mat img = data.imgSources[viewCam]->GetCurrent();
				
				
				// we can either make actual 3D things to render, or just project things to the image.
				// I'm inclined to do projection right now.
				
				//
				// Project the raw detections.
				// We'll draw those as some variant of pink.
				//
				PersonPose pers2d = pers3d.camPers[viewCam];
				for( unsigned jc = 0; jc < pers2d.joints.size(); ++jc )
				{
					hVec2D p = pers2d.joints[jc];
					float  c = pers2d.confidences[jc];
					
					int b,g,r;
					b = 256 * c;
					g = 0;
					r = 256 * c;
					
					cv::circle( img, cv::Point( p(0), p(1) ), 4, cv::Scalar( b,g,r ), 2 );
					
					
				}
				
				
				//
				// Project the reconstruction.
				//
				// We'll do that with cyan.
				//
				for( unsigned jc = 0; jc < pers3d.joints.size(); ++jc )
				{
					hVec2D p = data.occSettings.calibs[viewCam].Project( pers3d.joints[jc] );
					
					int b,g,r;
					b = 256;
					g = 256;
					r = 0;
					
					cv::circle( img, cv::Point( p(0), p(1) ), 4, cv::Scalar( b,g,r ), 2 );
					
					// wont hurt to draw a line from obs to recon
					hVec2D p2 = pers2d.joints[jc];
					cv::line( img, cv::Point( p(0), p(1) ), cv::Point( p2(0), p2(1) ), cv::Scalar( b/2, g/2, r/2 ), 2 );
				}
				
				ren->SetBGImage(img);
				
				bool done = !ren->Step(camChange, paused, frameAdvance);
				while( paused && frameAdvance == 0 )
				{
					done = !ren->Step(camChange, paused, frameAdvance);
				}
				if( camChange > 0 )
				{
					viewCam = std::min( viewCam+1, (int)data.imgSources.size()-1 );
				}
				if( camChange < 0 && viewCam > 0)
				{
					viewCam--;
				}
				camChange = 0;
				frameAdvance = 0;
				if( done )
					exit(0);
				data.imgSources[viewCam]->Advance();
			}
			
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
			pointNames[17] = "RIGHT_EAR";       pointNames[18] = "LEFT_EAR";
			  pointNames[15] = "RIGHT_EYE";  pointNames[16] = "LEFT_EYE";
			               pointNames[0] = "NOSE";
			               pointNames[1] = "NECK";
			pointNames[2] = "RIGHT_SHO";        pointNames[5] = "LEFT_SHO";
			pointNames[3] = "RIGHT_ELBOW";      pointNames[6] = "LEFT_ELBOW";
			pointNames[4] = "RIGHT_WRIST";      pointNames[7] = "LEFT_WRIST";
			               pointNames[8] = "MIDHIP";
			pointNames[9] = "RIGHT_HIP";        pointNames[12] = "LEFT_HIP";
			pointNames[10] = "RIGHT_KNEE";      pointNames[13] = "LEFT_KNEE";
			pointNames[11] = "RIGHT_ANKLE";     pointNames[14] = "LEFT_ANKLE";
			pointNames[24] = "RIGHT_HEEL";      pointNames[21] = "LEFT_HEEL";
			pointNames[22] = "RIGHT_MTP1";      pointNames[19] = "LEFT_MTP1";
			pointNames[23] = "RIGHT_MTP5";      pointNames[20] = "LEFT_MTP5";
		}
		else if( data.skelType == SKEL_APOSE )
		{
			pointNames[16] = "RIGHT_EAR";       pointNames[17] = "LEFT_EAR";
			  pointNames[14] = "RIGHT_EYE";  pointNames[15] = "LEFT_EYE";
			               pointNames[0] = "NOSE";
			               pointNames[1] = "NECK";
			pointNames[2] = "RIGHT_SHO";        pointNames[5] = "LEFT_SHO";
			pointNames[3] = "RIGHT_ELBOW";      pointNames[6] = "LEFT_ELBOW";
			pointNames[4] = "RIGHT_WRIST";      pointNames[7] = "LEFT_WRIST";

			pointNames[8] = "RIGHT_HIP";        pointNames[11] = "LEFT_HIP";
			pointNames[9] = "RIGHT_KNEE";       pointNames[12] = "LEFT_KNEE";
			pointNames[10] = "RIGHT_ANKLE";     pointNames[13] = "LEFT_ANKLE";
		}
		else if( data.skelType == SKEL_DLCUT )
		{
			               pointNames[13] = "FOREHEAD";
			               pointNames[12] = "CHIN";
			pointNames[8] = "RIGHT_SHO";        pointNames[9]  = "LEFT_SHO";
			pointNames[7] = "RIGHT_ELBOW";      pointNames[10] = "LEFT_ELBOW";
			pointNames[6] = "RIGHT_WRIST";      pointNames[11] = "LEFT_WRIST";

			pointNames[2] = "RIGHT_HIP";        pointNames[3]  = "LEFT_HIP";
			pointNames[1] = "RIGHT_KNEE";       pointNames[4]  = "LEFT_KNEE";
			pointNames[0] = "RIGHT_ANKLE";      pointNames[5]  = "LEFT_ANKLE";
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
					// NOTE: I would rather the residual was set to -1 here to indicate error, but setting to
					//       0 appears to work better in so far as our mc_opensim tools don't work when we 
					//       have -1 (and if we use -1 then we get lots of NANs when we load the file later)
					ezc3d::DataNS::Points3dNS::Point pt;
					pt.x( 0.0f );
					pt.y( 0.0f );
					pt.z( 0.0f );
					pt.residual(0.0f);
					
					pts.point(pt, name2idx[ pointNames[pc] ] );
				}
				
				ezc3d::DataNS::Frame f;
				f.add(pts);
				c3d.frame(f);
				cout << fc << " " << fc2 << " " << pts.nbPoints() << " empty " << endl;
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
						pt.residual(ji->second(3));
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
