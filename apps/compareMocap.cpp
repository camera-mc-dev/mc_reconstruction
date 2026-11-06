#include "imgio/imagesource.h"
#include "imgio/vidsrc.h"
#include "imgio/loadsave.h"
#include "imgio/vidWriter.h"

#include "math/mathTypes.h"
#include "math/intersections.h"
#include "math/matrixGenerators.h"

#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "renderer2/sdfText.h"
#include "renderer2/showImage.h"
#include "renderer2/renWrapper.h"

#include "misc/c3d.h"
#include "recon/poseFusion/poseFusion.h"

#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
using std::cout;
using std::endl;

#include "libconfig.h++"

#include "commonConfig/commonConfig.h"


struct STrackSource
{
	std::string trackFile;
	C3DData c3d;
	FrameOffsetter offsetter;
	
	bool hasSkeleton;
	std::string skeletonFile;
	Skeleton skeleton;
	
	bool hasOffset;
	int priorOffset;
};

struct SData
{
	std::string dataRoot;
	std::string testRoot;
	
	std::string offsetFile;
	int mocapOffset;
	
	std::vector<std::string> imgDirs;
	std::vector<std::string> vidFiles;
	std::vector<std::string> calibFiles;
	std::map<std::string,ImageSource*> sources;
	std::map<std::string,unsigned> camKey2Indx;
	
	std::vector< STrackSource > tracks;
	
	
	bool visualise;
	bool useHeadless;
	std::string headlessRenderOutput;
	bool outputToVideo;
	bool saveRender;
	
	unsigned long sourceTimestamp;
	double sourceFramerate;
	bool haveSourceTimestamp, haveSourceFramerate;
	
	
};



void ParseConfig( std::string configFile, SData &data );
void GetSources( SData &data );

class AlignRenderer : public Rendering::BasicRenderer
{
	friend class Rendering::RendererFactory;
	template<class T0, class T1 > friend class RenWrapper;
	// The constructor should be private or protected so that we are forced
	// to use the factory...
protected:
	// the constructor creates the renderer with a window of the specified
 	// size, and with the specified title.
	AlignRenderer(unsigned width, unsigned height, std::string title) : BasicRenderer(width,height,title) {}
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

int main(int argc, char* argv[])
{
	if( argc != 2 )
	{
		cout << "Tool to project .c3d files back to images" << endl;
		cout << "Usage:" << endl;
		cout << argv[0] << " <config file> " << endl;
		exit(0);
	}
	
	
	//
	// Find out where all our data sources are
	//
	SData data;
	ParseConfig( argv[1], data );
	
	
	//
	// Get image data
	//
	GetSources( data );
	
	
	
	//
	// Load / prepare the tracking data from .c3d files.
	//
	std::ofstream offsetsfi("frameOffsets");
	for( unsigned tfc = 0; tfc < data.tracks.size(); ++tfc )
	{
		STrackSource &ts = data.tracks[tfc];
		ts.c3d = C3DData( data.tracks[tfc].trackFile );
		
		// if we have timestamp and timecode we should use that - _but_ if user 
		// provides an offset it will override the the other data.
		if( !ts.hasOffset && data.haveSourceTimestamp && ts.c3d.GetTimeCode().valid )
		{
			cout << "Setting offset from timstamp / timecode" << endl;
			ts.offsetter = FrameOffsetter( data.sourceTimestamp, data.sourceFramerate,  ts.c3d.GetTimeCode(), ts.c3d.GetFramerate() );
		}
		else
		{
			if( !data.haveSourceFramerate && ts.hasOffset )
			{
				cout << "Setting offset using c3d framerate and prior offset" << endl;
				ts.offsetter = FrameOffsetter( ts.c3d.GetFramerate(), ts.c3d.GetFramerate(), ts.priorOffset );
			}
			else if( data.haveSourceFramerate && ts.hasOffset )
			{
				cout << "Setting offset using source and c3d framerates and prior offset" << endl;
				ts.offsetter = FrameOffsetter( data.sourceFramerate, ts.c3d.GetFramerate(), ts.priorOffset );
			}
			else
			{
				cout << "Offset assumed to be 0" << endl;
				// TODO: should check all .c3d files have the same framerate.
				// otherwise offset is default init as 0 offset same framerate as video source.
			}
		}
		
		offsetsfi << ts.trackFile << " : " << ts.offsetter.GetRatio() << " " << ts.offsetter.GetOffset() << endl;
	}
	offsetsfi.close();
	
	
	
	
	
	//
	// Map image source ids to indices.
	//
	bool done = false;
	std::map< int, std::string > ind2id;
	std::map< std::string, int > id2ind;
	int cind = 0;
	for( auto ci = data.sources.begin(); ci != data.sources.end(); ++ci )
	{
		ind2id[cind] = ci->first;
		id2ind[ ci->first ] = cind;
		++cind;
	}
	
	
	
	
	//
	// Create renderer
	//
	CommonConfig ccfg;
	
	// how big are the images?
	int r = data.sources.begin()->second->GetCalibration().height;
	int c = data.sources.begin()->second->GetCalibration().width;
	float ar = r/ (float) c;
	
	// create the renderer for display purposes.
	cout << "creating renderer..." << endl;
	std::shared_ptr<RenWrapper<AlignRenderer, Rendering::BasicHeadlessRenderer> > renWrapper;
	std::shared_ptr<VidWriter> vidWriter;
	float winW = ccfg.maxSingleWindowWidth;
	float winH = winW * ar;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	if( data.visualise )
	{
		renWrapper.reset( new RenWrapper<AlignRenderer, Rendering::BasicHeadlessRenderer>( data.useHeadless, winW, winH, "compare mocap" ) );
		
		if( data.useHeadless || data.saveRender )
		{
			boost::filesystem::path p( data.headlessRenderOutput );
			if( boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
			{
				// output is to a directory.
				data.outputToVideo = false;
			}
			else if(p.extension() == ".mp4")
			{
				data.outputToVideo = true;
				cv::Mat tmp( winH, winW, CV_8UC3, cv::Scalar(0,0,0) );
				if( data.haveSourceFramerate )
					vidWriter.reset( new VidWriter( data.headlessRenderOutput, "h264", tmp, data.sourceFramerate, 18, "yuv422p" ) );
				else
					vidWriter.reset( new VidWriter( data.headlessRenderOutput, "h264", tmp, 30, 18, "yuv422p" ) );
			}
			else
			{
				cout << "Headless output directory doesn't exist, or didn't recognise filename as .mp4 for video." << endl;
			}
		}
	}
	
	
	//
	// Create render node for 3D axes at scene origin
	//
	auto axesNode = Rendering::GenerateAxisNode3D( 500, "axesNode", renWrapper->GetActive() );
	renWrapper->Get3dRoot()->AddChild(axesNode);
	
	
	
	
	
	
	//
	// Run the rendering loop.
	//
	
	cind = 0;
	bool paused = true;
	int camChange = 0;
	int frameAdvance = 0;
	
	int curFrame = 0;
	
	
	cv::Mat img;
	while(!done)
	{
		curFrame = data.sources.begin()->second->GetCurrentFrameID();
		cout << "current image source frame: " << curFrame << endl;
		
		
		Calibration &calib = data.sources[ ind2id[cind] ]->GetCalibration();
		
		
		
		if( data.visualise )
		{
			img = data.sources[ ind2id[cind] ]->GetCurrent().clone();
			
			renWrapper->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			renWrapper->Get3dCamera()->SetFromCalibration( calib, 500, 15000 );
			renWrapper->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			
			for( unsigned trc = 0; trc < data.tracks.size(); ++trc )
			{
				STrackSource &ts = data.tracks[trc];
				
				// what track frame do we want?
				double mfc = ts.offsetter.FromReference( curFrame );
				cout << curFrame << " -> " << mfc << endl;
				
				// pick a colour for this track.
				cv::Scalar colour;
				switch( trc )
				{
					case 0: colour = cv::Scalar( 255,   0, 255 ); break;
					case 1: colour = cv::Scalar(   0, 255,   0 ); break;
					case 2: colour = cv::Scalar(   0,   0, 255 ); break;
					case 3: colour = cv::Scalar( 255, 255,   0 ); break;
					case 4: colour = cv::Scalar( 255,   0,   0 ); break;
					case 5: colour = cv::Scalar(   0, 255, 255 ); break;
				}
				
				
				// draw a circle for each point after projection.
				auto names = ts.c3d.GetPointNames();
				for( unsigned nc = 0; nc < names.size(); ++nc )
				{
					hVec3D p;
					
					if( ts.c3d.GetPointAtFrame( names[nc], mfc, p ) )
					{
						hVec2D p2 = calib.Project( p );
						cv::circle( img, cv::Point( p2(0), p2(1) ), std::max(8,img.rows/200), colour, 4 );
					}
				}
				
				//
				// Next, we need/want to draw the skeleton
				//
				if( ts.hasSkeleton )
				{
					colour = 0.7 * colour;
					
					
					auto pnames = ts.skeleton.GetNames();
					
					for( auto pni = pnames.begin(); pni != pnames.end(); ++pni )
					{
						std::string parent = ts.skeleton.GetParent( pni->second );
						
						hVec3D a,b;
						if( 
						    ts.c3d.GetPointAtFrame( pni->second, mfc, a ) &&
						    ts.c3d.GetPointAtFrame(      parent, mfc, b )
						  )
						{
							hVec2D a2 = calib.Project( a );
							hVec2D b2 = calib.Project( b );
							
							if( ts.skeleton.IsLeft( pni->second ) || ts.skeleton.IsLeft( parent ) )
								cv::line( img, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), colour, 2 );
							else
								cv::line( img, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), colour/2, 2 );
						}
						
						
					}
					
				}
			}
			
			renWrapper->SetBGImage(img);
			
			frameAdvance = 0;
			if( data.useHeadless )
				done = renWrapper->StepEventLoop();
			else
			{
				done = !renWrapper->ren->Step(camChange, paused, frameAdvance);
				while( paused && frameAdvance == 0 )
				{
					done = !renWrapper->ren->Step(camChange, paused, frameAdvance);
				}
			}
			cout << "cc, fa: " << camChange << " " << frameAdvance << " " << cind << endl;
			if( curFrame >= 0 && (!paused || (paused && frameAdvance == 1 )) )
				for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
					done = done || !sidi->second->Advance();
			
			if( camChange != 0 )
			{
				if( camChange > 0 && cind < ind2id.size()-1 )
				{
					++cind;
				}
				else if( camChange < 0 && cind > 0 )
					--cind;
				camChange = 0;
			}
			
			if( data.useHeadless || data.saveRender )
			{
				cv::Mat cap = renWrapper->Capture();
			
				if( data.outputToVideo )
				{
					cout << "Saving video file." << endl;
					vidWriter->Write( cap );
				}
				else
				{
					std::stringstream ss;
					ss << data.headlessRenderOutput << "/" << std::setw(6) << std::setfill('0') << data.sources.begin()->second->GetCurrentFrameID() << ".jpg";
					cout << ss.str() << endl;
					SaveImage( cap, ss.str() );
				}
				
			}
		}
		
		
		

	}
}

void ParseConfig( std::string configFile, SData &data )
{
	CommonConfig ccfg;

	data.dataRoot = ccfg.dataRoot;
	data.visualise = false;

	try
	{
		libconfig::Config cfg;
		cout << "read : " << configFile.c_str() << endl;
		cfg.readFile(configFile.c_str() );

		if( cfg.exists("dataRoot") )
			data.dataRoot = (const char*)cfg.lookup("dataRoot");
		data.testRoot = (const char*)cfg.lookup("testRoot");

		if( cfg.exists("imgDirs" ) )
		{
			libconfig::Setting &idirs = cfg.lookup("imgDirs");
			for( unsigned ic = 0; ic < idirs.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) idirs[ic];
				data.imgDirs.push_back(s);
			}
		}

		if( cfg.exists("vidFiles" ) )
		{
			libconfig::Setting &vidfs = cfg.lookup("vidFiles");
			for( unsigned ic = 0; ic < vidfs.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) vidfs[ic];
				data.vidFiles.push_back(s);
			}

			libconfig::Setting &cfiles = cfg.lookup("calibFiles");
			for( unsigned ic = 0; ic < cfiles.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) cfiles[ic];
				data.calibFiles.push_back(s);
			}
		}

		libconfig::Setting &trks = cfg.lookup("trackData");
		for( unsigned tfc = 0; tfc < trks.getLength(); ++tfc )
		{
			STrackSource ts;
			
			std::stringstream ss;
			ss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) trks[tfc].lookup("trackFile");
			ts.trackFile = ss.str();
			
			if( trks[tfc].exists("frameOffset") )
			{
				ts.priorOffset = trks[tfc].lookup("frameOffset");
				ts.hasOffset   = true;
			}
			else
				ts.hasOffset = false;
			
			if( trks[tfc].exists("skeletonFile") )
			{
				ts.hasSkeleton = true;
				ss.str("");
				ss << data.dataRoot << data.testRoot << (const char*) trks[tfc].lookup("skeletonFile");
				ts.skeletonFile = ss.str();
				ts.skeleton = Skeleton( ss.str() );
			}
			else
				ts.hasSkeleton = false;
			
			data.tracks.push_back( ts );
		}
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
			data.useHeadless = false;
			data.saveRender  = false;
			if( cfg.exists("renderHeadless") )
			{
				data.useHeadless = cfg.lookup("renderHeadless");
				data.headlessRenderOutput = data.dataRoot + data.testRoot + (const char*) cfg.lookup("renderTarget");
			}
			
			if( !data.useHeadless && cfg.exists("renderTarget" ) )
			{
				data.saveRender = true;
				data.headlessRenderOutput = data.dataRoot + data.testRoot + (const char*) cfg.lookup("renderTarget");
			}
		}
		
		
		
		if( cfg.exists("imgSourceTimestamp") )
		{
			data.sourceTimestamp = cfg.lookup("imgSourceTimestamp");
			data.haveSourceTimestamp = true;
		}
		else
		{
			data.sourceTimestamp = 0;
			data.haveSourceTimestamp = false;
		}
		
		if( cfg.exists("imgSourceFramerate") )
		{
			data.sourceFramerate = cfg.lookup("imgSourceFramerate");
			data.haveSourceFramerate = true;
		}
		else
		{
			data.sourceFramerate = 0.0;
			data.haveSourceFramerate = false;
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

void GetSources( SData &data )
{
	std::string camKey;
	if( data.imgDirs.size() > 0 )
	{
		for( unsigned ic = 0; ic < data.imgDirs.size(); ++ic )
		{
			int a = data.imgDirs[ic].rfind( "/", data.imgDirs[ic].size()-2 );
			int b = data.imgDirs[ic].find( "/", a+1 );

			camKey = std::string( data.imgDirs[ic].begin() + a+1, data.imgDirs[ic].begin() + b );

			cout << "creating source: " << data.imgDirs[ic] << " (camKey: " << camKey << ")" << endl;
			data.sources[camKey] = (ImageSource*) new ImageDirectory(data.imgDirs[ic]) ;

			data.camKey2Indx[camKey] = ic;
		}
	}
	else if( data.vidFiles.size() > 0 )
	{
		for( unsigned ic = 0; ic < data.vidFiles.size(); ++ic )
		{
			int a = data.vidFiles[ic].rfind( "/", data.vidFiles[ic].size()-2 )+1;
			int b = data.vidFiles[ic].find( ".", a );
			camKey = std::string( data.vidFiles[ic].begin() + a, data.vidFiles[ic].begin() + b );

			cout << "creating source: " << data.vidFiles[ic] << " with " << data.calibFiles[ic] << " (camKey: " << camKey << ")" << endl;
			data.sources[camKey] = (ImageSource*) new VideoSource(data.vidFiles[ic], data.calibFiles[ic]);

			data.camKey2Indx[camKey] = ic;
		}
	}
	else
	{
		cout << "no data.imgDirs nor data.vidFiles." << endl;
		exit(0);
	}
}
