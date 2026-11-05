#ifndef C3D_LOADER_H
#define C3D_LOADER_H

#ifdef USE_EZC3D

#include <map>
#include <vector>
#include <string>
#include "math/mathTypes.h"



class C3DData
{
public:
	
	C3DData(std::string filename);
	~C3DData();
	
	bool GetPointAtFrame( std::string pointName, float frame, hVec3D &point);
	bool GetAnalogsAtFrame(float frame, Eigen::VectorXf &data);
	
	std::vector< std::string > GetPointNames(  )
	{
		std::vector< std::string > pointNames;
		for( auto pi = points.begin(); pi != points.end(); ++pi )
			pointNames.push_back( pi->first );
		return pointNames;
	}
	unsigned GetStartFrame()
	{
		return startFrame ;
	}
	
	unsigned GetEndFrame()
	{
		return startFrame + numFrames;
	}
	
	unsigned GetNumAnalogChannels()
	{
		return channelNames.size();
	}
	
	struct STimeCode
	{
		STimeCode() { valid = false; }
		
		std::vector< int > timecode;
		unsigned subFrames;
		unsigned offset;
		unsigned codeRate;
		
		bool valid;
	};
	
	float GetFramerate(){ return framerate; }
	STimeCode GetTimeCode() { return timecode; }
	
private:
	void InterpolatePointData(float frame, std::string pointName, hVec3D &point);
	void InterpolateAnalogData(float frame, std::string channelName, float &value);
	
	// each point gets its own matrix
	std::map<std::string, genMatrix> points;
	
	// all channels in one matrix
	genMatrix channels;
	std::vector< std::string > channelNames;
	
	
	// frame meta data
	unsigned startFrame;
	unsigned numFrames;
	unsigned analogSubframes;
	float    framerate;
	
	
	// timecode data (if available)
	STimeCode timecode;
	
	std::string filePath;
};

#endif

#endif
