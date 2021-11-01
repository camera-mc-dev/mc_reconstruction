#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "renderer2/sdfText.h"

#include "imgio/vidsrc.h"
#include "imgio/loadsave.h"

#include "misc/tokeniser.h"

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
using std::cout;
using std::endl;

typedef std::map< std::string, std::map< std::string, hVec2D > > frame_t;

int main(int argc, char *argv[] )
{
	std::string trialPath( argv[1] );
	
	std::map<std::string,VideoSource*> sources;
	
	sources["00"] = new VideoSource(trialPath + "/00.mp4", "none");
	sources["01"] = new VideoSource(trialPath + "/01.mp4", "none");
	sources["02"] = new VideoSource(trialPath + "/02.mp4", "none");
	sources["03"] = new VideoSource(trialPath + "/03.mp4", "none");
	sources["04"] = new VideoSource(trialPath + "/04.mp4", "none");
	sources["05"] = new VideoSource(trialPath + "/05.mp4", "none");
	sources["06"] = new VideoSource(trialPath + "/06.mp4", "none");
	sources["07"] = new VideoSource(trialPath + "/07.mp4", "none");
	sources["08"] = new VideoSource(trialPath + "/08.mp4", "none");
	
	std::ifstream infi( trialPath + "/markers2D");
	
	std::map< int, frame_t > frames;
	while(infi)
	{
		std::string l;
		std::getline(infi, l);
		if( !infi )
			continue;
		
		std::vector< std::string > ss;
		ss = SplitLine( l, " \t");
		int vfc = atoi( ss[0].c_str() );
		int mfc = atoi( ss[1].c_str() );
		
		
		frame_t perCamPartPoints; // [cam][part]
		for( unsigned tc = 2; tc < ss.size(); tc += 5 )
		{
			std::string cam  = ss[tc+0];
			std::string part = ss[tc+1];
			hVec2D p;
			p << atof(ss[tc+2].c_str()), atof(ss[tc+3].c_str()), atof(ss[tc+4].c_str());
			
			perCamPartPoints[cam][part] = p;
		}
		
		frames[vfc] = perCamPartPoints;
	}
	
	bool done = false;
	cv::Mat i;
	while( !done )
	{
		for( auto si = sources.begin(); si != sources.end(); ++si )
		{
			int fc = si->second->GetCurrentFrameID();
			i = si->second->GetCurrent();
			
			cout << fc << " " << si->first; 
			
			auto fi = frames.find( fc );
			if( fi != frames.end() )
			{
				cout << " gf ";
				frame_t &f = fi->second;
				auto ci = f.find( si->first );
				if( ci != f.end() )
				{
					cout << " gc " << ci->second.size();
					for( auto pi = ci->second.begin(); pi != ci->second.end(); ++pi )
					{
						cv::circle( i, cv::Point( pi->second(0), pi->second(1) ), 4, cv::Scalar(255,255,0), 2 );
					}
				}
				else
				{
					for( auto ci = f.begin(); ci != f.end(); ++ci )
					{
						cout << " [ " << ci->first << " ] ";
					}
					cout << endl;
					exit(0);
				}
			}
			cout << endl;
			
			std::stringstream ss;
			ss << "m2drender_" << si->first << "/" << std::setw(6) << std::setfill('0') << fc << ".jpg";
			SaveImage( i, ss.str() );
			
			done = done || !si->second->Advance();
		}
	}
}
