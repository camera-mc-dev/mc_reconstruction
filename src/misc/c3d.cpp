#ifdef USE_EZC3D


#include "misc/c3d.h"

#include <ezc3d/ezc3d.h>
#include <ezc3d/Header.h>
#include <ezc3d/Data.h>

#include <iostream>
using std::cout;
using std::endl;

void LoadC3DFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points )
{
	genMatrix channels;
	LoadC3DFile( filename, startFrame, points, channels );
}

void LoadC3DFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points, genMatrix &channels )
{
	ezc3d::c3d c3d( filename );
	
	// c3d has 3 main accessors:
	// c3d.header()
	// c3d.parameters()
	// c3d.data()
	//
	// other useful things:
	// c3d.pointNames() returns a vector of strings of point names.
	// c3d.channelNames()
	// c3d.channelIdx()
	
	
	// probably the first thing that we want to know is the 
	// names of all the markers.
	// technically, these are not just markers, but any 
	// 3D point.
	auto names = c3d.pointNames();
	
	// There are channels as well, which might contain analog signals.
	auto cnames = c3d.channelNames();
	
	int numAnalogs = c3d.header().nbAnalogs();
	
	cout << "\t\tc3d reader: names: " << names.size() << "  cnames: " << cnames.size() << " numAnalogs: " << numAnalogs << endl;
	cout << "\t\t            names: ";
	for( unsigned nc = 0; nc < names.size(); ++nc )
	{
		cout << names[nc] << " ";
	}
	cout << endl;
	cout << "\t\t            cnames: ";
	for( unsigned nc = 0; nc < cnames.size(); ++nc )
	{
		cout << cnames[nc] << " ";
	}
	cout << endl;
	
	
	//
	// Now let's enquire about the frame data.
	//
	unsigned numFrames  = c3d.header().nbFrames();
	unsigned firstFrame = c3d.header().firstFrame();
	cout << "\t\t            nframes, fframe: " << numFrames << " " << firstFrame << endl;
	
	//
	// Which means we can create an empty points matrix
	// of the correct size for each point.
	//
	// the matrix will store x,y,z,1.0, r, v
	// where r is the residual, and v is 1 for valid or 0 for not valid.
	//
	for( unsigned nc = 0; nc < names.size(); ++nc )
	{
		points[ names[nc] ] = genMatrix::Zero(6, numFrames + firstFrame);
	}
	
	// I'd like to think that we could now make much the same structure for the 
	// analog data - but we might find that this is not general enough for all circumstances.
	unsigned analogSubframes = c3d.header().nbAnalogByFrame();
	unsigned numAnalogFrames = (numFrames + firstFrame) * analogSubframes;
	channels = genMatrix::Zero( cnames.size(), numAnalogFrames);
	
	
	//
	// So we can now access each of the frames and fill
	// our data matrices.
	//
	for( unsigned fc = 0; fc < numFrames; ++fc )
	{
		for( unsigned nc = 0; nc < names.size(); ++nc )
		{
			hVec3D p; p << c3d.data().frame(fc).points().point(nc).x(), 
			               c3d.data().frame(fc).points().point(nc).y(), 
			               c3d.data().frame(fc).points().point(nc).z(), 
			               1.0f;
			
			float  r = c3d.data().frame(fc).points().point(nc).residual();
// 			bool   v = c3d.data().frame(fc).points().point(nc).residual() > 0 && 
// 			           !c3d.data().frame(fc).points().point(nc).isEmpty();
			bool   v = !c3d.data().frame(fc).points().point(nc).isEmpty();
			
			if( v )
				points[ names[nc] ].col(fc + firstFrame) << p(0), p(1), p(2), p(3),  r, 1.0f;
			else
				points[ names[nc] ].col(fc + firstFrame) << p(0), p(1), p(2), p(3),  r, 0.0f;
		}
		
		for( unsigned nc = 0; nc < cnames.size(); ++nc )
		{
			for( unsigned sfc = 0; sfc < analogSubframes; ++sfc )
			{
				// NOTE: I should probably do more checks to make sure that this frame has the number of subframes 
				//       that we are expecting.
				int idx = (firstFrame*analogSubframes) + (fc*analogSubframes) + sfc;
				channels(nc, idx) = c3d.data().frame(fc).analogs().subframe(sfc).channel(nc).data();
			}
		}
	}
	
	startFrame = firstFrame;
}

#endif
