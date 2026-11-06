#include "misc/c3d.h"
#include "misc/tokeniser.h"

#include <iostream>
#include <string>
#include <vector>


unsigned CalculateFrameOffset( C3DData::STimecode reference, std::vector<unsigned> targetCode, unsigned targetSubframes )
{
	//returns the number of "subframes" required to move from the reference timecode to the target timecode
}

unsigned long AdvanceTimestampToFrame( unsigned long inputTimestamp, double framerate, unsigned targetFrame )
{
	// assuming the inputTimestamp corresponds to frame 0 of some video,
    // is microseconds since the unix epoch, and the framerate is Hz,
	// return the timecode of the targetFrame.
}


int main( int argc, char* argv[] )
{
	if( argc != 4 )
	{
		cout << "Identify cutting frames for a video file such that they align with the given timecodes, with reference to the given .c3d file" << endl;
		cout << "Usage: " << endl;
		cout << argv[0] << " <video timestamp (microseconds)>  <video framerate>   <referece .c3d path>  <start timecode> <end timecode>" << endl;
		cout << endl;
		cout << "Note: Timecodes should include 'subframes' as hrs:mins:secs:frames|subframes" << endl;
		exit(0);
	}
	
	
	unsigned long timeStamp = std::atoi( argv[1] );
	double        framerate = std::atod( argv[2] );
	
	std::string c3dpath( argv[3] );
	std::string startCodeStr( argv[4] );
	std::string endCodeStr( argv[5] );
	
	
	//
	// Split codes and convert to ints
	//
	std::vector<unsigned> startCode, endCode;
	unsigned subFramesStart = 0;
	unsigned subFramesEnd   = 0;
	auto sc = SplitLine( startCodeStr, ":|" );
	auto ec = SplitLine( startCodeStr, ":|" );
	
	assert( len(sc) == 5 );
	assert( len(ec) == 5 );
	for( unsigned c = 0; c < 4; ++c )
	{
		startCode[c]  = std::atoi( sc[c] );
		endCode[c]    = std::atoi( ec[c] );
	}
	subFramesStart = std::atoi( sc.back() );
	subFramesEnd   = std::atoi( ec.back() );
	
	
	
	//
	// Load c3d file and initialise frame offsetter
	//
	C3DData c3d( c3dpath );
	FrameOffsetter fo( timeStamp,  framerate,  c3d.timecode, c3d.framerate );
	
	
	//
	// Using the c3d.timecode as reference, find which c3d frame corresponds to the provided timecodes.
	//
	unsigned c3dFrameStart = CalculateFrameOffset( c3d.timecode, startCode, subFramesStart );
	unsigned c3dFrameEnd   = CalculateFrameOffset( c3d.timecode,   endCode,   subFramesEnd );
	
	
	//
	// Find out the video frame of the start and end timecodes
	//
	unsigned vidFrameStart = fo.ToReference( c3dFrameStart );
	unsigned vidFrameEnd   = fo.ToReference(   c3dFrameEnd );
	
	
	//
	// Calculate the timestamp at the new start frame
	//
	unsigned long newTimestamp = AdvanceTimestampToFrame( timestamp, framerate, vidFrameStart );
	
	
	
	cout << "------------" << endl;
	cout << "new start frame: " << vidFrameStart << endl;
	cout << "new   end frame: " << vidFrameEnd   << endl;
	cout << "new timestamp  : " << newTimestamp  << endl;
	
	
}
