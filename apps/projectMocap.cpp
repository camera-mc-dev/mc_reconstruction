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
#include "misc/tokeniser.h"

#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
using std::cout;
using std::endl;

#include "libconfig.h++"

#include "commonConfig/commonConfig.h"

// only a total idiot would accidentally use this as the name of the events file.
#define NO_EVENT_FILE "-$noEvents$-"


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
	
	std::string eventsFile;
	std::map< int, std::vector< std::string > > events;
	
	
	std::vector< std::string > trackFiles;
	std::vector<unsigned> trackStartFrames;
	std::map< std::string, genMatrix > tracks;
	genMatrix channels;
	
	// render nodes.
	std::shared_ptr< Rendering::MeshNode > imgNode;
	std::shared_ptr< Rendering::SceneNode > tlRoot;
	std::shared_ptr< Rendering::MeshNode > momentNode;
	std::shared_ptr< Rendering::MeshNode > hmNode;
	std::map< int, std::shared_ptr< Rendering::SceneNode > > eventNodes;
	std::shared_ptr< Rendering::SceneNode > txtRoot;
	Rendering::SDFText *textMaker;
	
	cv::Mat channelsHeatmap;
	
	
	std::string renderTarget;
	bool renderHeadless;
	
	bool visualise;
	
};


class AlignRenderer : public Rendering::BasicRenderer
{
	friend class Rendering::RendererFactory;	
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

void ParseConfig( std::string configFile, SData &data );
void GetSources( SData &data );
void ReadEvents( SData &data );
void CreateNodes( std::shared_ptr< Rendering::AbstractRenderer > ren, SData &data, int winW, int winH, int timeLineWidth, int numVidFrames, std::shared_ptr<Rendering::SceneNode> bgRoot, std::shared_ptr<Rendering::SceneNode> fgRoot );

int main(int argc, char* argv[])
{
	if( argc != 2 )
	{
		cout << "Tool to project .c3d or .trc files back to images" << endl;
		cout << "Usage:" << endl;
		cout << argv[0] << " <config file> " << endl;
		exit(0);
	}
	
	SData data;
	ParseConfig( argv[1], data );
	GetSources( data );
	ReadEvents( data );
	
	//
	// Create renderer
	//
	CommonConfig ccfg;
	
	// how big are the images?
	int r = data.sources.begin()->second->GetCalibration().height;
	int c = data.sources.begin()->second->GetCalibration().width;
	float ar = r/ (float) c;
	
	// create the renderer for display purposes.
	cout << "creating renderer" << endl;
	
	float winW = ccfg.maxSingleWindowWidth;
	float timeLineWidth = 100;
	float winH = (winW - timeLineWidth) * ar;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	
	//
	// Load the c3d track files.
	//
	data.trackStartFrames.assign( data.trackFiles.size(), 0 );
	for( auto tfc = 0; tfc < data.trackFiles.size(); ++tfc )
	{
		std::map< std::string, genMatrix > newTracks;
		genMatrix newChannels;
		
		if( data.trackFiles[tfc].find(".c3d") != std::string::npos )
		{
			cout << "loading (c3d): " << data.trackFiles[tfc] << endl;
			LoadC3DFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks, newChannels );
		}
		else if( data.trackFiles[tfc].find(".trc") > 0 != std::string::npos )
		{
			cout << "loading (trc): " << data.trackFiles[tfc] << endl;
			LoadTRCFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks );
			
// 			transMatrix3D Ry = RotMatrix( 0, 1, 0, -3.14 );
// 			transMatrix3D Sy = transMatrix3D::Identity();
// 			Sy(1,1)          = -1.0f;
// 			transMatrix3D Tr  = transMatrix3D::Identity();
// 			Tr(1,3) = 300;
// 			transMatrix3D M  = transMatrix3D::Identity();
// 			for( auto ti = newTracks.begin(); ti != newTracks.end(); ++ti )
// 			{
// 				ti->second.block(0, 0, 4, ti->second.cols()) = M * ti->second.block(0, 0, 4, ti->second.cols());
// 			}
			
		}
		cout << "\tnew tracks: " << newTracks.size() << endl;
		cout << "\tstart Frame: " << data.trackStartFrames[tfc] << endl;
		for( auto ti = newTracks.begin(); ti != newTracks.end(); ++ti )
		{
			
			std::string name = ti->first;
			if( data.trackFiles[tfc].find("op-") != std::string::npos )
				name = "op-" + name;
			if( data.trackFiles[tfc].find("ap-") != std::string::npos )
				name = "ap-" + name;
			if( data.trackFiles[tfc].find("dlc-") != std::string::npos )
				name = "dlc-" + name;
			cout << "\t" << ti->first << " ( " << name << " ) " << endl;
			if( data.tracks.find( name ) == data.tracks.end() )
			{
				data.tracks[ name ] = ti->second;
			}
			else
			{
				cout << "Track " << ti->first << " ( " << name << " ) " << " from " << data.trackFiles[tfc] << " already exists from other file?" << endl;
				exit(0);
			}
		}
		
		if( data.channels.cols() > 0 && data.channels.cols() != newChannels.cols() )
		{
			cout << "ignoring channels from file: " << data.trackFiles[tfc] << " because it has different frames from previous channels data " << endl;
			cout << newChannels.cols() << " vs: " << data.channels.cols() << endl;
		}
		else if( data.channels.cols() > 0 )
		{
			genMatrix tmp( data.channels.rows() + newChannels.rows(), data.channels.cols() );
			tmp << data.channels, newChannels;
			
			data.channels = tmp;
		}
		else
			data.channels = newChannels;
	}
	
	cout << "track names: " << endl;
	unsigned long minElements = 99999999;
	for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
	{
		minElements = std::min( minElements, (unsigned long)ti->second.cols() );
		cout << "\t" << ti->first << " " << ti->second.cols() << endl;
	}
	cout << "min elements: " << minElements << endl;
	
	cout << "channels: " << data.channels.rows() << " " << data.channels.cols();
	if( data.channels.rows() > 0 && data.channels.cols() > 0 )
    {
		 cout << " with max value: " << data.channels.maxCoeff() << endl;
		// let's turn the channels data into a simple heatmap.
		// I want there to be as many rows of this image as there are _video_ frames, except multiplied by
		// the fact that we have 1000 Hz instead of 200 Hz
		// TODO: Handle when it is not 200 Hz and 1000 Hz
		// 
		int numHMRows = data.sources.begin()->second->GetNumImages() * 5;
		cout << numHMRows << " " << data.sources.begin()->second->GetNumImages() << " " << data.channels.cols() << endl;
		cv::Mat hmch( numHMRows, data.channels.rows(), CV_32FC3, cv::Scalar(0) );
		data.channels /= data.channels.maxCoeff();
		for( unsigned rc = 0; rc < std::min(hmch.rows, (int)data.channels.cols()); ++rc )
		{
			for( unsigned cc = 0; cc < hmch.cols; ++cc )
			{
				float x = data.channels(cc,rc);
				cv::Vec3f &p = hmch.at<cv::Vec3f>(rc,cc);
				
				p[2] = 1 - (0.5 + 0.5 * cos(  x   * 3.14) );
				p[1] = 1 - (0.5 + 0.5 * cos(  x   * 6.28) );
				p[0] = 1 - (0.5 + 0.5 * cos((x+1) * 3.14) );
			}
		}
		data.channelsHeatmap = hmch;
		SaveImage( hmch, "channelsMap.jpg" );
	}
	else
	{
		cout << " -> no data" << endl;
		data.channelsHeatmap = cv::Mat( 1, data.channels.rows(), CV_32FC3, cv::Scalar(0) );
	}
	
	
	std::shared_ptr< RenWrapper<AlignRenderer, Rendering::BasicHeadlessRenderer> > renWrapper;
	if( data.visualise )
	{
		renWrapper.reset( new RenWrapper<AlignRenderer, Rendering::BasicHeadlessRenderer> (data.renderHeadless, winW, winH, "mocap vis" ));
		CreateNodes( renWrapper->GetActive(), data, winW, winH, timeLineWidth, data.sources.begin()->second->GetNumImages(), renWrapper->Get2dBgRoot(), renWrapper->Get2dFgRoot() );
		
		renWrapper->Get2dBgCamera()->SetOrthoProjection(0, winW, 0, winH, -10, 10);
		renWrapper->Get2dFgCamera()->SetOrthoProjection(0, winW, 0, winH, -10, 10);
	}
	
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
	
	cout << data.offsetFile << endl;
	if( data.offsetFile.compare("NO_OFFSET_FILE") == 0 )
	{
		data.mocapOffset = 0;
	}
	else
	{
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
	// 		eo = -9; // fuck it...
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
	
	
	// are we outputting to video or images?
	std::shared_ptr<VidWriter> vidWriter;
	bool outputToVideo = false;
	if( data.renderHeadless )
	{
		boost::filesystem::path p( data.renderTarget );
		if( boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
		{
			// output is to a directory.
			outputToVideo = false;
		}
		else if(p.extension() == ".mp4")
		{
			outputToVideo = true;
			cv::Mat tmp( winH, winW, CV_8UC3, cv::Scalar(0,0,0) );
			vidWriter.reset( new VidWriter( data.renderTarget, "h264", tmp, 25, 18, "yuv422p" ) );
		}
		else
		{
			cout << "Headless output directory doesn't exist, or didn't recognise filename as .mp4 for video." << endl;
		}
	}
	
	
	std::ofstream ptest("ptest");
	hVec3D p,o,x;
	o << 0,0,0,1.0f;
	x << 1,0,0,1.0f;
	for( float y = -3000; y < 3001; y += 500 )
	{
		p << 0, y, 1000, 1.0f;
		ptest << p.transpose() << endl;
		for( auto si = data.sources.begin(); si != data.sources.end(); ++si )
		{
			ptest << si->first << " : ";
			hVec3D co = si->second->GetCalibration().TransformToWorld(o);
			hVec3D cx = si->second->GetCalibration().TransformToWorld(x);
			hVec3D d  = cx - co;
			
			hVec3D p2  = p;
			hVec2D pa = si->second->GetCalibration().Project( p );
			float n;
			do
			{
				p2 += d;
				hVec2D p2a = si->second->GetCalibration().Project( p2 );
				n = (p2a-pa).norm();
			}
			while( n < 1.0f );
			
			ptest << (p2-p).norm() << endl;
			
			
		}
	}
	
	
	
	std::stringstream oss;
	oss << data.dataRoot << "/" << data.testRoot << "/markers2D";
	std::ofstream outfi( oss.str() );
	
	cind = 0;
	bool paused = true;
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
			
			hVec3D O,X,Y,Z;
			O << 0  ,0  ,  0,1.0f;  hVec2D o = calib.Project( O );
			X << 500,0  ,  0,1.0f;  hVec2D x = calib.Project( X );
			Y << 0  ,500,  0,1.0f;  hVec2D y = calib.Project( Y );
			Z << 0  ,0  ,500,1.0f;  hVec2D z = calib.Project( Z );
			cv::line( img, cv::Point( o(0), o(1) ), cv::Point( x(0), x(1) ), cv::Scalar(  0,  0,255), 2 );
			cv::line( img, cv::Point( o(0), o(1) ), cv::Point( y(0), y(1) ), cv::Scalar(  0,255,0), 2 );
			cv::line( img, cv::Point( o(0), o(1) ), cv::Point( z(0), z(1) ), cv::Scalar(255,  0,0), 2 );
			
			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
			{
				if( mfc-2 < 0 || mfc+2 >=  ti->second.cols() )
					continue;
				
				hVec2D a = calib.Project( ti->second.col( mfc-2  ).head(4) );
				hVec2D b = calib.Project( ti->second.col( mfc-1  ).head(4) );
				hVec2D c = calib.Project( ti->second.col( mfc    ).head(4) );
				hVec2D d = calib.Project( ti->second.col( mfc+1  ).head(4) );
				hVec2D e = calib.Project( ti->second.col( mfc+2  ).head(4) );
				
				cv::line( img, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), cv::Scalar(128,  0,   0), 2);
				cv::line( img, cv::Point( b(0), b(1) ), cv::Point( c(0), c(1) ), cv::Scalar(255,  0,   0), 2);
// 				cv::line( img, cv::Point( c(0), c(1) ), cv::Point( d(0), d(1) ), cv::Scalar(  0,  0, 255), 2);
// 				cv::line( img, cv::Point( d(0), d(1) ), cv::Point( e(0), e(1) ), cv::Scalar(  0,  0, 128), 2);
				
				
				if( ti->first.find("Neck") != std::string::npos )
				{
					cout << ti->second.col( mfc    ).transpose() << endl;
					cout << b.transpose() << endl;
					cout << c.transpose() << endl;
					cout << d.transpose() << endl;
				}
				
			}
			
			data.imgNode->GetTexture()->UploadImage( img );
			transMatrix3D T;
			T = transMatrix3D::Identity();
			T(1,3) =  (float)vfc / data.sources[ ind2id[cind] ]->GetNumImages() * winH;
			data.momentNode->SetTransformation(T);
			
			auto ei = data.events.find( mfc );
			if( ei != data.events.end() )
			{
				cout << "====================" << endl;
				cout << " events: ";
				for( unsigned c = 0; c < ei->second.size(); ++c )
				{
					cout << ei->second[c] << " ";
				}
				cout << endl;
				cout << "====================" << endl;
			}
			
			for( auto eni = data.eventNodes.begin(); eni != data.eventNodes.end(); ++eni )
			{
				std::stringstream ss;
				ss << eni->second->GetID() << "-line";
				std::shared_ptr< Rendering::MeshNode > li = std::dynamic_pointer_cast<Rendering::MeshNode>( eni->second->FindChild( ss.str() ) );
				
				if( vfc == eni->first )
				{
					Eigen::Vector4f g; g << 0.0f, 1.0f, 0.0f, 1.0f;
					li->SetBaseColour( g );
					
					data.txtRoot->Clean();
					hVec2D txtPos; txtPos << winW - timeLineWidth - winH/100 * 8, (float)vfc / data.sources.begin()->second->GetNumImages() * winH, 1.0f;
					Eigen::Vector4f tcol; tcol << 1.0, 1.0, 1.0, 1.0f;
					data.textMaker->RenderString( data.events[eni->first][0], winH/30, txtPos, tcol, data.txtRoot );
				}
				else if( vfc > eni->first )
				{
					Eigen::Vector4f b; b << 0.0f, 0.0f, 1.0f, 1.0f;
					li->SetBaseColour( b );
				}
			}
			
			
			
			
			frameAdvance = 0;
			if( data.renderHeadless )
			{
				done = renWrapper->StepEventLoop();
				frameAdvance = 1;
			}
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
					++cind;
				else if( camChange < 0 && cind > 0 )
					--cind;
				camChange = 0;
			}
			
			// capture and save (if wanted)
			if( data.renderHeadless )
			{
				cv::Mat grab = renWrapper->Capture();
				if( outputToVideo )
				{
					vidWriter->Write( grab );
				}
				else
				{
					std::stringstream ss;
					ss << data.renderTarget << "/" << std::setw(6) << std::setfill('0') << vfc << ".jpg";
					SaveImage( grab, ss.str() );
				}
			}
			
		}
		
		if( mfc >= 0 && mfc <  data.tracks.begin()->second.cols() )
		{
			outfi << vfc << " " << mfc << "\t";
			
			
			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
			{
				if( mfc < ti->second.cols() )
				{
					for( unsigned cind2 = 0; cind2 < data.sources.size(); ++cind2 )
					{
						Calibration &calib = data.sources[ ind2id[cind2] ]->GetCalibration();
						hVec2D a = calib.Project( ti->second.col( mfc ).head(4) );
						outfi << ind2id[cind2] << " ";
						if( ti->second( 5, mfc ) > 0 )
						{
							outfi << ti->first << " " << a.transpose() << "\t";
						}
						else
						{
							if( ti->first.find("MIDPOINT") != std::string::npos )
							{
								cout << ti->first << " " << ti->second.col(mfc).transpose() << endl;
								exit(0);
							}
							outfi << ti->first << " 0 0 0\t";
						}
					}
				}
			}
			outfi << endl;
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
		
		if( cfg.exists("eventsFile") )
		{
			std::stringstream ss;
			ss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) cfg.lookup("eventsFile");;
			data.eventsFile = ss.str();
		}
		else
		{
			data.eventsFile = NO_EVENT_FILE;
		}
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
			
			data.renderHeadless = false;
			if( cfg.exists("renderHeadless" ) )
			{
				data.renderHeadless = cfg.lookup("renderHeadless");
				data.renderTarget   = data.dataRoot + data.testRoot + (const char*)cfg.lookup("renderTarget");
			}
		}
		
		if( cfg.exists("offsetFile") )
		{
			data.offsetFile = data.dataRoot + data.testRoot + (const char*) cfg.lookup("offsetFile");
		}
		else
		{
			data.offsetFile = "NO_OFFSET_FILE";
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


void ReadEvents( SData &data )
{
	// we will assume the events file has entries in _frames_ not in _times_
	// This is a strange and horrid file format, but, what ho!
	if( data.eventsFile.compare( NO_EVENT_FILE ) != 0 )
	{
		std::ifstream infi( data.eventsFile );
		if( !infi )
		{
			std::stringstream ss;
			ss << "Couldn't open events file: " << data.eventsFile << endl;
			throw std::runtime_error( ss.str() );
		}
		
		std::vector< std::vector< std::string > > lineTokens;
		std::string line;
		while( std::getline(infi, line) )
		{
			lineTokens.push_back( SplitLine( line, " " ) );
		}
		
		// I think we should have 4 + 1 lines of column headers, and then the rest are data rows.
		assert( lineTokens.size() >= 5 );
		
		// parse each data row.
		for( unsigned lc = 5; lc < lineTokens.size(); ++lc )
		{
			// don't need the item number in the first column.
			for( unsigned sc = 1; sc < lineTokens[lc].size(); ++sc )
			{
				// data rows should have the "item" column which header rows don't have.
				assert( lineTokens[lc].size() == lineTokens[1].size() + 1 );
				
				// column name?
				std::string columnName = lineTokens[1][sc-1];
				
				// column value?
				std::string valStr = lineTokens[lc][sc];
				if( valStr.compare("NaN") != 0 )
				{
					int frame = std::atof( valStr.c_str() );
					
					data.events[ frame ].push_back( columnName );
					
					cout << "got event: " << frame << " " << columnName << endl;
				}
			}
		}
	}
}

void CreateNodes( std::shared_ptr< Rendering::AbstractRenderer > ren, SData &data, int winW, int winH, int timeLineWidth, int numVidFrames, std::shared_ptr<Rendering::SceneNode> bgRoot, std::shared_ptr<Rendering::SceneNode> fgRoot )
{
	data.imgNode = Rendering::GenerateImageNode(0, 0, winW - timeLineWidth, winH, "imgNode", ren );
	bgRoot->AddChild( data.imgNode );
	
	
	// timeline root is just an offset to the side of our window.
	Rendering::NodeFactory::Create(data.tlRoot, "timelineRoot");
	transMatrix3D T = transMatrix3D::Identity();
	T(0,3) = winW - timeLineWidth;
	data.tlRoot->SetTransformation(T);
	
	// then for each event, we want some mark on the timeline.
	for( auto ei = data.events.begin(); ei != data.events.end(); ++ei )
	{
		std::stringstream ss;
		ss << "enode-" << ei->first;
		Rendering::NodeFactory::Create(data.eventNodes[ ei->first ], ss.str());
		
// 		// make a text label ? No... makes it too busy on the screen
// 		for( unsigned c = 0; c < ei->second.size(); ++c )
// 		{
// 			
// 		}
		
		// make a line.
		ss << "-line";
		std::vector< Eigen::Vector2f > pts(2);
		float h = (float)ei->first / (float) numVidFrames * winH; // TODO: change 2000 to number of frames in video.
		pts[0] << 0, h;
		pts[1] << timeLineWidth, h;
		auto n = Rendering::GenerateLineNode2D( pts, 2, ss.str(), ren );
		data.eventNodes[ ei->first ]->AddChild(n);
		
		Eigen::Vector4f red; red << 1.0f, 0.0f, 0.0f, 1.0f;
		n->SetBaseColour( red );
		
		data.tlRoot->AddChild( data.eventNodes[ ei->first ] );
	}
	
	// then our progress through the timeline.
	hVec3D centre;
	centre << winW - timeLineWidth/2.0, -(0.5*winH), -5.0f, 1.0f;
	data.momentNode = GenerateRectNode(centre, timeLineWidth, winH, 0.1, "momentNode", ren);
	
	Eigen::Vector4f progCol; progCol << 0.4f, 0.8f, 0.8f, 0.6f;
	data.momentNode->SetBaseColour(progCol);
	bgRoot->AddChild( data.momentNode );
	
	fgRoot->AddChild( data.tlRoot );
	
	// and a node for the event name
	Rendering::NodeFactory::Create(data.txtRoot, "txtRoot");
	fgRoot->AddChild( data.txtRoot );
	
	// and something to write with
	CommonConfig ccfg;
	std::stringstream fntss;
	fntss << ccfg.coreDataRoot << "/NotoMono-Regular.ttf";
	data.textMaker = new Rendering::SDFText (fntss.str(), ren);
	
	// some way of showing the signals from the analog channels.
	data.hmNode = Rendering::GenerateImageNode(winW-timeLineWidth, 0, timeLineWidth, winH, data.channelsHeatmap, "hmNode", ren );
	bgRoot->AddChild( data.hmNode );
	
// 	std::vector< std::shared_ptr< Rendering::MeshNode > > eventNodes;
}
