#include "imgio/imagesource.h"
#include "imgio/vidsrc.h"
#include "imgio/loadsave.h"
#include "math/mathTypes.h"
#include "math/intersections.h"
#include "math/matrixGenerators.h"

#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "renderer2/sdfText.h"
#include "renderer2/showImage.h"

#include "misc/c3d.h"

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
	
	bool visualise;
	
};


void ParseConfig( std::string configFile, SData &data );
void GetSources( SData &data );

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
	cout << "creating window" << endl;
	std::shared_ptr<AlignRenderer> ren;
	float winW = ccfg.maxSingleWindowWidth;
	float winH = winW * ar;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	if( data.visualise )
		Rendering::RendererFactory::Create( ren, winW, winH, "mocap vis");
	
	//
	// Load the c3d track files.
	//
	data.trackStartFrames.assign( data.trackFiles.size(), 0 );
	for( auto tfc = 0; tfc < data.trackFiles.size(); ++tfc )
	{
		cout << "loading: " << data.trackFiles[tfc] << endl;
		std::map< std::string, genMatrix > newTracks;
		LoadC3DFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks );
		cout << "\tnew tracks: " << newTracks.size() << endl;
		cout << "\tstart Frame: " << data.trackStartFrames[tfc] << endl;
		for( auto ti = newTracks.begin(); ti != newTracks.end(); ++ti )
		{
			
			std::string name = ti->first;
			if( data.trackFiles[tfc].find("op-fused") != std::string::npos )
				name = "op-" + name;
			if( data.trackFiles[tfc].find("ap-fused") != std::string::npos )
				name = "ap-" + name;
			if( data.trackFiles[tfc].find("dlc-fused") != std::string::npos )
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
	
	
	
	
	std::stringstream oss;
	oss << data.dataRoot << "/" << data.testRoot << "/markers2D";
	std::ofstream outfi( oss.str() );
	
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
			
			ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			ren->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			
			
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
				cv::line( img, cv::Point( c(0), c(1) ), cv::Point( d(0), d(1) ), cv::Scalar(  0,  0, 255), 2);
				cv::line( img, cv::Point( d(0), d(1) ), cv::Point( e(0), e(1) ), cv::Scalar(  0,  0, 128), 2);
			}
		
		
		
			ren->SetBGImage(img);
			
			frameAdvance = 0;
			done = !ren->Step(camChange, paused, frameAdvance);
			while( paused && frameAdvance == 0 )
			{
				done = !ren->Step(camChange, paused, frameAdvance);
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
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
		}
		
		data.offsetFile = data.dataRoot + data.testRoot + (const char*) cfg.lookup("offsetFile");
		
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
