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

class FrameOffseter
{
public:
	FrameOffseter(  )
	{
		offset = 0;
		ratio  = 1;
	}
	FrameOffseter( int refFrameRate,  int othRate, int othOffsetAtRef0 )
	{
		offset = othOffsetAtRef0;
		ratio  = othRate / refFrameRate;
	}

	FrameOffseter( unsigned long refTimestamp,  int refRate,  C3DData::STimeCode othTimeCode, double othRate, unsigned leapSeconds=37 )
	{
		//
		// Now this gets... interesting. We _could_ be dealing with up to 3 different time formats:
		//  1) video timestamp + framerate
		//  2) c3d data framerate
		//  3) c3d timecode + timecode rate.
		//
		
		// use chrono and adjust timestamp to get as microseconds since midnight
		uint64_t ts = refTimestamp - (uint64_t)(leapSeconds * 1'000'000LL);
		uint64_t microSecondsInDay = 86400LL /*seconds*/ * 1'000'000LL;
		uint64_t tsMicroSecondsSinceMidnight =  ts % microSecondsInDay;

		// convert microseconds since midnight into mocap subframes since midnight.
		uint64_t usPerSubframe = 1'000'000 / (uint64_t)(othTimeCode.codeRate * othTimeCode.subFrames);
		uint64_t tsSubframes = tsMicroSecondsSinceMidnight / usPerSubframe;

		// get timecode as subframes since midnight
		uint64_t tcSeconds =   (uint64_t)othTimeCode.timecode[0] * 3600LL
		                     + (uint64_t)othTimeCode.timecode[1] * 60LL
		                     + (uint64_t)othTimeCode.timecode[2];
		uint64_t tcSubframes = (tcSeconds * othTimeCode.codeRate + (othTimeCode.timecode[3]+1)) * othTimeCode.subFrames   + othTimeCode.offset;

		double tcOffset = tsSubframes - tcSubframes;
		double tcRate   = othTimeCode.codeRate * othTimeCode.subFrames;
		double tcRatio  = refRate / tcRate;

		offset = tcRatio * tcOffset;
		ratio  = othRate / refRate;

		// offset -= 2;

		// cout << othTimeCode.timecode[0] << ":"  << othTimeCode.timecode[1] << ":" << othTimeCode.timecode[2] << ":" << othTimeCode.timecode[3] << " " << endl;
		// cout << othTimeCode.offset << endl;
		// cout << refTimestamp << endl;

		// cout << tsSubframes << endl;
		// cout << tcSubframes << endl;
		// cout << tsSubframes - tcSubframes << endl;
		// cout << tcRatio << endl;
		// cout << "---" << endl;
		// cout << offset << endl;
		// cout << ratio << endl;
	}

	double ToReference( double frame )
	{
		// given frame in our space, move to reference.
		return frame/ratio - offset;
	}

	double FromReference( double frame )
	{
		return (frame + offset) * ratio;
	}

private:

	double offset;
	double ratio;

};

#endif

#endif
