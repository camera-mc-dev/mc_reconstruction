#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"

#include "misc/c3d.h"

#include <fstream>
#include <iostream>
using std::cout;
using std::endl;




//
// The job of this program is to load up sparse pose detections from 
// multiple cameras, as well as a fused reconstruction, and create 
// a simple visualisation of how much error there is for each viewpoint
// vs. the reconstruction.
//

class PCBGRenderer : public Rendering::BasicRenderer
{
	friend class Rendering::RendererFactory;
	template<class T0, class T1 > friend class RenWrapper;
	
	PCBGRenderer(unsigned width, unsigned height, std::string title) : BasicRenderer(width,height,title) {}
	
	void FinishConstructor()
	{
		BasicRenderer::FinishConstructor();
		std::stringstream ss;
		ss << ccfg.shadersRoot << "/";
		LoadVertexShader(ss.str() + "pcloudVertex.glsl", "pcloudVertex");
		CreateShaderProgram("pcloudVertex", "colourFrag", "pcloudShader");
		
	}
};


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
	Skeleton skeleton;
	
	
	
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
		
		std::string s = (const char*)cfg.lookup("poseDataType");
		if( s.compare("jsonDir") == 0 )  // directory of JSON files
		{
			data.poseDataType = POSE_JSON_DIR;
		}
		else if( s.compare("jsonMMPose") == 0 )
		{
			data.poseDataType = POSE_JSON_MMPOSE;
		}
		else if( s.compare("dlccsv") == 0 )
		{
			data.poseDataType = POSE_DLC_CSV;
		}
		else
		{
			cout << "can only load pose data from jsonDir or jsonMMPose or dlccsv" << endl;
			cout << "got: " << s << endl;
			exit(0);
		}
		
		s = (const char*)cfg.lookup("skeletonFile");
		data.skeleton = Skeleton( s );
		
		
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
				if( !infi )
				{
					cout << "Could not open specified offset file: " << s ;
					exit(0);
				}
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
		switch( data.poseDataType )
		{
			case POSE_JSON_DIR:
				ReadPoseDirJSON( data.poseSources[sc], data.pcPoses[sc] );
				break;
			
			case POSE_JSON_MMPOSE:
				ReadPoseMMPoseJSON( data.poseSources[sc], data.pcPoses[sc] );
				break;
			
			
			case POSE_DLC_CSV:
				ReadDLC_CSV( data.poseSources[sc], data.pcPoses[sc] );
				break;
		}
	}
}





int main( int argc, char* argv[] )
{
	if( argc != 3 )
	{
		cout << argv[0] << " tool to vis per-view error vs. reconstruction " << endl;
		cout << "Usage:" << endl;
		cout << argv[0] << " <config file> < .c3d track file > " << endl;
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
	// Load up the reconstruction
	//
	std::stringstream ss;
	std::map< std::string, genMatrix > tracks;
	unsigned trackStartFrame;
	cout << "loading (c3d): " << argv[2] << endl;
	LoadC3DFile( argv[2], trackStartFrame, tracks );
	
	unsigned trackLength = 0;
	for( auto ti = tracks.begin(); ti != tracks.end(); ++ti )
	{
		cout << ti->first << " : " << ti->second.rows() << " " << ti->second.cols() << endl;
		trackLength = std::max( trackLength, (unsigned)ti->second.cols() );
	}
	
	
	//
	// We'll create a renderer that shows a top-down view of the scene - kind of.
	//
	
	// find out the positions of the cameras and the max AA distance from origin
	float maxAAD;
	maxAAD = 0;
	std::vector< hVec3D > camCents;
	for( unsigned cc = 0; cc < data.occSettings.calibs.size(); ++cc )
	{
		hVec3D cent = data.occSettings.calibs[cc].GetCameraCentre();
		
		camCents.push_back( cent );
		
		maxAAD = std::max( maxAAD, std::abs( cent(0) ) );
		maxAAD = std::max( maxAAD, std::abs( cent(1) ) );
		
		cout << cc << " : " << cent.transpose() << endl;
	}
	
	cout << "maxAAD : " << maxAAD << endl;
	std::shared_ptr< PCBGRenderer > ren;
	
	
	// create the renderer for display purposes.
	cout << "creating window" << endl;
	CommonConfig ccfg;
	float winW = std::min( ccfg.maxSingleWindowWidth, ccfg.maxSingleWindowHeight );
	Rendering::RendererFactory::Create( ren, winW, winW, "vperr vis");
	
	maxAAD *= 1.1;
	ren->Get2dBgCamera()->SetOrthoProjection( -maxAAD, maxAAD, -maxAAD, maxAAD, -maxAAD, maxAAD);
	ren->Get2dFgCamera()->SetOrthoProjection( -maxAAD, maxAAD, -maxAAD, maxAAD, -maxAAD, maxAAD);
	
	// our squished wireframe cube for a camera.
	std::shared_ptr<Rendering::Mesh> camCube;
	hVec3D o; o << 0,0,0,1;
	camCube = Rendering::GenerateCube(o, 0.1);
	float cna, cnb;
	cna = 20.0f;
	cnb = 80.0f;
	camCube->vertices <<  -cna, -cna,  cna,  cna,    -cnb,-cnb, cnb,  cnb,
	                      -cna,  cna,  cna, -cna,    -cnb, cnb, cnb, -cnb,
	                         0,    0,    0,    0,     cnb, cnb, cnb,  cnb,
	                         1,    1,    1,    1,      1,    1,   1,    1;
	camCube->UploadToRenderer(ren);
	
	cv::Mat nullImg(10,10, CV_8UC3, cv::Scalar(0,0,0) );
	auto nullTex = std::make_shared<Rendering::Texture>( Rendering::Texture(ren) );
	nullTex->UploadImage(nullImg);
	
	Eigen::Vector4f camCol; camCol << 1.0, 1.0, 1.0, 1.0;
	std::vector< std::shared_ptr<Rendering::MeshNode> > camNodes( data.occSettings.calibs.size() );
	for( unsigned cc = 0; cc < data.occSettings.calibs.size(); ++cc )
	{
		std::stringstream ss;
		ss << "nodeForCamera_" << cc;
		Rendering::NodeFactory::Create(camNodes[cc], ss.str() );
		camNodes[cc]->SetMesh( camCube );
		camNodes[cc]->SetBaseColour(camCol);
		camNodes[cc]->SetTexture(nullTex);
		camNodes[cc]->SetTransformation( data.occSettings.calibs[cc].L.inverse() );
		camNodes[cc]->SetShader( ren->GetShaderProg("basicColourShader"));
		
		ren->Get2dFgRoot()->AddChild( camNodes[cc] );
	}
	
	
	
	// then create a simple point cloud mesh for the track.
	std::shared_ptr<Rendering::Mesh> trackMesh( new Rendering::Mesh( tracks.size(),0 ) ); // lots of verts, no faces.
	std::shared_ptr< Rendering::MeshNode > trackMeshNode;
	Rendering::NodeFactory::Create( trackMeshNode, "trackMesh" );
	
	
	trackMeshNode->SetShader( ren->GetShaderProg("pcloudShader") );
	trackMeshNode->SetTexture( ren->GetBlankTexture() );
	
	Eigen::Vector4f black; black << 0.0f, 0.0f, 0.0f, 0.0f;
	trackMeshNode->SetBaseColour(black);
	
	trackMesh->UploadToRenderer(ren);
	trackMeshNode->SetMesh( trackMesh );
	
	ren->Get2dFgRoot()->AddChild( trackMeshNode );
	
	for( unsigned fc = trackStartFrame; fc < trackStartFrame + trackLength; ++fc )
	{
		unsigned vc = 0;
		for( auto ti = tracks.begin(); ti != tracks.end(); ++ti )
		{
			trackMesh->vertices.col( vc ).head(3) = ti->second.col( vc ).head(3);
			trackMesh->vertColours.col( vc ) << 1.0, 1.0, 1.0, 1.0;
			++vc;
		}
		
		trackMesh->UploadToRenderer(ren);
		trackMeshNode->SetMesh( trackMesh );
		
		
		//
		// Go through each camera, compute the "error" and set the colour of the camera 
		// node as appropriate.
		//
		for( unsigned cc = 0; cc < data.occSettings.calibs.size(); ++cc )
		{
			float error = 0.0f;
			for( unsigned personIdx = 0; personIdx < pcPoses[ cc ][ fc ].size(); ++personIdx )
			{
				PersonPose &person = pcPoses[ cc ][ fc ][pc];
				
				for( auto ti = tracks.begin(); ti != tracks.end(); ++ti )
				{
					// this track is ti->first
					// find the right point in the person
					// compute distance between point and projection of track
					// accumulate error.
					// NOTE: compute error vs each person and take smallest.
				}
			}
			
			
			hVec3D errCol; 
			camNodes[cc]->SetBaseColour(errCol);
		}
		
		
		ren->StepEventLoop();
	}
	
	
	/*
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
					// NOTE: seems to be possible to have a view with no observations? Yes, of course it is.
					if( pers2d.joints.size() == pers3d.joints.size() )
					{
						hVec2D p2 = pers2d.joints[jc];
						if( (p2(0) > 0 || p2(1) > 0) && p2(2) > data.minConf ) // don't draw line to bad obs.
							cv::line( img, cv::Point( p(0), p(1) ), cv::Point( p2(0), p2(1) ), cv::Scalar( b/2, g/2, r/2 ), 2 );
					}
					
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
	}*/
}


