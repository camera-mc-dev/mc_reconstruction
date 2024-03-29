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
#include "misc/trcLoader.h"
#include "recon/poseFusion/poseFusion.h"

#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
using std::cout;
using std::endl;

#include "libconfig.h++"

#include "commonConfig/commonConfig.h"




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
	
	
	std::vector< std::string > trackFiles;
	std::vector<unsigned> trackStartFrames;
	std::map< std::string, genMatrix > tracks;
	
	// these must be in same order as trackFiles
	std::vector< std::string > skeletonFiles; 
	std::vector< poseSource_t > poseDataTypes;
	std::vector< Skeleton > skeletons;
	
	bool visualise;
	bool useHeadless;
	std::string headlessRenderOutput;
	bool outputToVideo;
	
};

bool GetLine( SData &data, int tfc, int mfc, std::string n0, std::string n1, cv::Point &p0, cv::Point &p1)
{
	if( mfc < 0 )
		return false;
	std::stringstream ss0;
	ss0 << std::setw(2) << std::setfill('0') << tfc << "-" << n0;
	
	std::stringstream ss1;
	ss1 << std::setw(2) << std::setfill('0') << tfc << "-" << n1;
	
	auto t0 = data.tracks.find( ss0.str() );
	auto t1 = data.tracks.find( ss1.str() );
	
	if( t0 == data.tracks.end() || t1 == data.tracks.end() )
		return false;
	
	
	Calibration &calib = data.sources.begin()->second->GetCalibration();
	hVec2D h0 = calib.Project( t0->second.col( mfc ).head(4) );
	hVec2D h1 = calib.Project( t1->second.col( mfc ).head(4) );
	
	p0 = cv::Point( h0(0), h0(1) );
	p1 = cv::Point( h1(0), h1(1) );
	
	return true;
}


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
	
	SData data;
	ParseConfig( argv[1], data );
	GetSources( data );
	
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
			vidWriter.reset( new VidWriter( data.headlessRenderOutput, "h264", tmp, 25, 18, "yuv422p" ) );
		}
		else
		{
			cout << "Headless output directory doesn't exist, or didn't recognise filename as .mp4 for video." << endl;
		}
	}
	
	//
	// Load the c3d track files.
	//
	data.trackStartFrames.assign( data.trackFiles.size(), 0 );
	for( auto tfc = 0; tfc < data.trackFiles.size(); ++tfc )
	{
		cout << "loading: " << data.trackFiles[tfc] << endl;
		std::map< std::string, genMatrix > newTracks;
		if( data.trackFiles[tfc].find(".c3d") != std::string::npos )
		{
			cout << "loading (c3d): " << data.trackFiles[tfc] << endl;
			LoadC3DFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks );
		}
		else if( data.trackFiles[tfc].find(".trc") > 0 != std::string::npos )
		{
			cout << "loading (trc): " << data.trackFiles[tfc] << endl;
			LoadTRCFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks );
		}
		cout << "\tnew tracks: " << newTracks.size() << endl;
		cout << "\tstart Frame: " << data.trackStartFrames[tfc] << endl;
		for( auto ti = newTracks.begin(); ti != newTracks.end(); ++ti )
		{
			std::stringstream ss;
			ss << std::setw(2) << std::setfill('0') << tfc << "-" << ti->first;
			cout << ss.str() << endl;
			data.tracks[ ss.str() ] = ti->second;
			
		}
	}
	
	cout << "track names: " << endl;
	unsigned long minElements = 99999999;
	for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
	{
		minElements = std::min( minElements, (unsigned long)ti->second.cols() );
		cout << "\t" << ti->first << " " << ti->second.cols() << endl;
	}
	cout << "min elements: " << minElements << endl;
	
	
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
	
	
	if( data.offsetFile.compare("None") == 0 )
	{
		data.mocapOffset = 0;
	}
	else
	{
		cout << data.offsetFile << endl;
		std::ifstream infi( data.offsetFile );
		std::string type;
		infi >> type;
		if( type.compare("offset:") == 0 )
		{
			infi >> data.mocapOffset;
			cout << type << " " << data.mocapOffset << endl;
		}
		else if( type.compare("extraOffset:") == 0 )
		{
			int eo;
			infi >> eo;
// 			eo = -9; // fuck it...
			data.mocapOffset = minElements - data.sources.begin()->second->GetNumImages() + eo;
			cout << type << " " << eo << " ( " << data.mocapOffset << " ) " << endl;
			
			infi.close();
			std::ofstream outfi(data.offsetFile);
			
			outfi << "offset: " << data.mocapOffset << endl;
			outfi << "extraOffset: " << eo << endl;
			outfi << endl;
			outfi << "-------------" << endl;
			outfi << "offset set manually using extraOffset that was most consistent over dataset" << endl << endl;
		}
	}
	
	
	auto axesNode = Rendering::GenerateAxisNode3D( 500, "axesNode", renWrapper->GetActive() );
	renWrapper->Get3dRoot()->AddChild(axesNode);
	
	cind = 0;
	bool paused = false;
	int camChange = 0;
	int frameAdvance = 0;
	
	int mfc = std::min(0, data.mocapOffset);
	cv::Mat img;
	while(!done)
	{
		int vfc = mfc - data.mocapOffset;
		if( data.visualise )
		{
			int fc = data.sources.begin()->second->GetCurrentFrameID();
			cout << endl << vfc << " (" << fc << ") " << mfc << " " << data.tracks.begin()->second.cols() << endl;
		}
		else
		{
			cout << endl << vfc << " " << mfc << " " << data.tracks.begin()->second.cols() << endl;
		}
		
		
		
		Calibration &calib = data.sources[ ind2id[cind] ]->GetCalibration();
		
		if( data.visualise )
		{
			img = data.sources[ ind2id[cind] ]->GetCurrent().clone();
			
			renWrapper->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			renWrapper->Get3dCamera()->SetFromCalibration( calib, 500, 15000 );
			renWrapper->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			
			
			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
			{
				if( mfc-2 < 0 || mfc+2 >=  ti->second.cols() )
					continue;
				
				int a = ti->first.find("-");
				int n = atoi( std::string( ti->first.begin(), ti->first.begin()+a ).c_str() );
				
				cv::Scalar colour;
				switch( n )
				{
					case 0: colour = cv::Scalar( 255,   0, 255 ); break;
					case 1: colour = cv::Scalar(   0, 255,   0 ); break;
					case 2: colour = cv::Scalar(   0,   0, 255 ); break;
					case 3: colour = cv::Scalar( 255, 255,   0 ); break;
					case 4: colour = cv::Scalar( 255,   0,   0 ); break;
					case 5: colour = cv::Scalar(   0, 255, 255 ); break;
				}
				
				hVec2D c = calib.Project( ti->second.col( mfc    ).head(4) );
				
				cv::circle( img, cv::Point( c(0), c(1) ), 4, colour, 3 );
			}
			
			
			for( auto tfc = 0; tfc < data.trackFiles.size(); ++tfc )
			{
				cv::Scalar colour;
				switch( tfc )
				{
					case 0: colour = cv::Scalar( 255,   0, 255 ); break;
					case 1: colour = cv::Scalar(   0, 255,   0 ); break;
					case 2: colour = cv::Scalar(   0,   0, 255 ); break;
					case 3: colour = cv::Scalar( 255, 255,   0 ); break;
					case 4: colour = cv::Scalar( 255,   0,   0 ); break;
					case 5: colour = cv::Scalar(   0, 255, 255 ); break;
				}
				std::stringstream ss;
				
				cv::Point a, b;
				auto pnames = data.skeletons[tfc].GetNames();
				for( auto i = pnames.begin(); i != pnames.end(); ++i )
				{
					std::string parent = data.skeletons[tfc].GetParent( i->second );
					
					if( GetLine( data, tfc, mfc, i->second, parent, a, b) )
					{
						if( data.skeletons[tfc].IsLeft( i->second ) || data.skeletons[tfc].IsLeft( parent ) )
							cv::line( img, a, b, colour, 2 );
						else
							cv::line( img, a, b, colour/2, 2 );
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
			if( vfc >= 0 && (!paused || (paused && frameAdvance == 1 )) )
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
			
			if( data.useHeadless )
			{
				cv::Mat cap = renWrapper->Capture();
				
				if( data.outputToVideo )
				{
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
		
		
		++mfc;
		done = done || mfc >= data.tracks.begin()->second.cols();
		
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
		
		
		libconfig::Setting &trkfs = cfg.lookup("trackFiles");
		for( unsigned tfc = 0; tfc < trkfs.getLength(); ++tfc )
		{
			std::stringstream ss;
			ss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) trkfs[tfc];
			data.trackFiles.push_back( ss.str() );
		}
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
			data.useHeadless = false;
			if( cfg.exists("renderHeadless") )
			{
				data.useHeadless = cfg.lookup("renderHeadless");
				data.headlessRenderOutput = data.dataRoot + data.testRoot + (const char*) cfg.lookup("renderTarget");
			}
		}
		
		data.offsetFile = "None";
		if( cfg.exists("offsetFile") )
			data.offsetFile = data.dataRoot + data.testRoot + (const char*) cfg.lookup("offsetFile");
		
		
		libconfig::Setting &skfs = cfg.lookup("skeletonFiles");
		assert( skfs.getLength() == data.trackFiles.size() );
		for( unsigned c = 0; c < skfs.getLength(); ++c )
		{
			std::string s = (const char*) skfs[c];
			std::stringstream ss;
			ss << data.dataRoot << data.testRoot << s;
			data.skeletonFiles.push_back( ss.str() );
			data.skeletons.push_back( Skeleton( s ) );
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
