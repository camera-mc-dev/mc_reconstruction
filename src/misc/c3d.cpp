#ifdef USE_EZC3D

#include "misc/c3d.h"

#include <ezc3d/ezc3d.h>
#include <ezc3d/Header.h>
#include <ezc3d/Data.h>
#include <ezc3d/Parameters.h>
#include <ezc3d/Parameter.h>

#include <iostream>
#include <cmath>
using std::cout;
using std::endl;




C3DData::C3DData(std::string filePath)
{
	ezc3d::c3d c3d( filePath );
	
	
	//
	// Relevant meta-data
	//
	auto names = c3d.pointNames();
	channelNames = c3d.channelNames();
	
	int numAnalogs = c3d.header().nbAnalogs();
	assert( channelNames.size() == numAnalogs );
	
	numFrames  = c3d.header().nbFrames();
	startFrame = c3d.header().firstFrame();
	framerate  = c3d.header().frameRate();
	
	if( c3d.parameters().isGroup("TIMECODE") )
	{
		timecode.valid = true;
		
		auto g = c3d.parameters().group("TIMECODE");
		//g.print();
		
		auto standard = g.parameter("STANDARD").valuesAsString()[0];
		if( standard.compare("PAL") == 0 )
		{
			timecode.codeRate = 25;
		}
		else
			throw std::runtime_error( "Can only handle PAL (25 Hz) in c3d TIMECODE right now" );
		
		timecode.offset = g.parameter("OFFSETS").valuesAsInt()[0];
		timecode.timecode = g.parameter("TIMECODES").valuesAsInt();
		timecode.subFrames = g.parameter("SUBFRAMESPERFRAME").valuesAsInt()[0];
		
		cout << filePath << " had timecode " << timecode.timecode[0] << ":" << 
		                                        timecode.timecode[1] << ":" << 
		                                        timecode.timecode[2] << ":" <<
		                                        timecode.timecode[3] << "|" <<
		                                        timecode.offset      << endl;
	}
	else
		timecode.valid = false;
	
	
	//
	// point data is x,y,z,1.0, residual, valid
	//
	for( unsigned nc = 0; nc < names.size(); ++nc )
	{
		cout << names[nc] << numFrames << endl;
		points[ names[nc] ] = genMatrix::Zero(6, numFrames );
	}
	
	analogSubframes = c3d.header().nbAnalogByFrame();
	unsigned numAnalogFrames = 0;
	numAnalogFrames = numFrames * analogSubframes;
	channels = genMatrix::Zero( channelNames.size(), numAnalogFrames);
	
	for( unsigned fc = 0; fc < numFrames; ++fc )
	{
		for( unsigned nc = 0; nc < names.size(); ++nc )
		{
			hVec3D p; p << c3d.data().frame(fc).points().point(nc).x(),
			               c3d.data().frame(fc).points().point(nc).y(),
			               c3d.data().frame(fc).points().point(nc).z(),
			               1.0f;
			
			float  r = c3d.data().frame(fc).points().point(nc).residual();
			bool   v = !c3d.data().frame(fc).points().point(nc).isEmpty();
			
			if( v )
				points[ names[nc] ].col(fc) << p(0), p(1), p(2), p(3),  r, 1.0f;
			else
				points[ names[nc] ].col(fc) << p(0), p(1), p(2), p(3),  r, 0.0f;
		}
		
		for( unsigned nc = 0; nc < channelNames.size(); ++nc )
		{
			for( unsigned sfc = 0; sfc < analogSubframes; ++sfc )
			{
				int idx = (fc*analogSubframes) + sfc;
				channels(nc, idx) = c3d.data().frame(fc).analogs().subframe(sfc).channel(nc).data();
			}
		}
	}
}

C3DData::~C3DData()
{
}



bool C3DData::GetPointAtFrame( std::string pointName, float frame, hVec3D &point)
{
	float f = frame - startFrame;
	
	auto p = points.find( pointName );
	if( p == points.end() || f < 0 || f > numFrames-1 )
	{
		point << 0,0,0,0;
		return false;
	}
	
	unsigned f0 = (unsigned) std::floor( f );
	unsigned f1 = (unsigned) std::ceil( f );
	
	f0 = std::max( 0u, f0 );
	f1 = std::min( numFrames-1, f1 );
	
	// TODO: Check validity column and fix interpolation as needed
	hVec3D p0 = p->second.col( f0 ).head(4);
	hVec3D p1 = p->second.col( f1 ).head(4);
	
	float alpha = f - f0;
	
	point = p0 + alpha * p1;
	
	return true;
}

bool C3DData::GetAnalogsAtFrame(float frame, Eigen::VectorXf &data)
{
	float f = frame - startFrame;
	if( f < 0 )
		return false;
	
	float subFrame = f * analogSubframes;
	int f0 = (int) std::floor( subFrame );
	int f1 = (int) std::ceil( subFrame );
	
	f0 = std::max( 0, f0 );
	f1 = std::min( (int)channels.cols()-1, f1 );
	
	Eigen::VectorXf c0 = channels.col( f0 );
	Eigen::VectorXf c1 = channels.col( f1 );
	
	float alpha = subFrame - f0;
	data = c0 + alpha * c1;
	
	return true;
}


#endif
