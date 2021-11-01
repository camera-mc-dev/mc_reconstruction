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
	
	// There are channels as well, but I am ignoring those for now as
	// I don't think they match my uses.
	auto cnames = c3d.channelNames();
	
	cout << "\t\tc3d reader: names: " << names.size() << "  cnames: " << cnames.size() << endl;
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
	}
	
	startFrame = firstFrame;
}

#endif
