#include "misc/c3d.h"
#include "misc/tokeniser.h"

#include <iostream>
#include <string>
#include <vector>

unsigned CalculateFrameOffset(C3DData::STimeCode reference, std::vector<unsigned> targetCode, unsigned targetOffset)
{
    // Calculate the number of "subframes" required to move from the reference timecode to the target timecode
	
	// Convert reference timecode to subframes
    auto referenceFrames   = reference.codeRate * 
	                         (   reference.timecode[0] * 3600 
	                           + reference.timecode[1] * 60 
	                           + reference.timecode[2]         ) + reference.timecode[3];
	auto referenceSubframes = referenceFrames * reference.subFrames + reference.offset;
	
	// Convert target timecode to subframes
	auto targetFrames   = reference.codeRate * 
	                         (   targetCode[0] * 3600 
	                           + targetCode[1] * 60 
	                           + targetCode[2]         ) + targetCode[3];
	auto targetSubframes = targetFrames * reference.subFrames + targetOffset;
	
	// Calculate the difference in subframes
	return targetSubframes - referenceSubframes;
}

unsigned long AdvanceTimestampToFrame(unsigned long inputTimestamp, double framerate, unsigned targetFrame)
{
    // Calculate the duration in microseconds for the target frame
    unsigned long durationMicroseconds = static_cast<unsigned long>(targetFrame * (1.0 / framerate) * 1e6);

    // Add the duration to the input timestamp
    return inputTimestamp + durationMicroseconds;
}

int main(int argc, char* argv[])
{
    if (argc != 6)
    {
        std::cout << "Identify cutting frames for a video file such that they align with the given timecodes, with reference to the given .c3d file" << std::endl;
        std::cout << "Usage: " << std::endl;
        std::cout << argv[0] << " <video timestamp (microseconds)>  <video framerate>   <reference .c3d path>  <start timecode> <end timecode>" << std::endl;
        std::cout << std::endl;
        std::cout << "Note: Timecodes should include 'subframes' as hrs:mins:secs:frames|subframes" << std::endl;
        exit(0);
    }

    unsigned long timeStamp = std::atoi(argv[1]);
    double framerate = std::atof(argv[2]);

    std::string c3dpath(argv[3]);
    std::string startCodeStr(argv[4]);
    std::string endCodeStr(argv[5]);

    // Split codes and convert to ints
    std::vector<unsigned> startCode, endCode;
    unsigned subFramesStart = 0;
    unsigned subFramesEnd = 0;
    auto sc = SplitLine(startCodeStr, ":|");
    auto ec = SplitLine(endCodeStr, ":|");

    assert(sc.size() == 5);
    assert(ec.size() == 5);
    for (unsigned c = 0; c < 4; ++c)
    {
        startCode.push_back(std::atoi(sc[c].c_str()));
        endCode.push_back(std::atoi(ec[c].c_str()));
    }
    subFramesStart = std::atoi(sc.back().c_str());
    subFramesEnd = std::atoi(ec.back().c_str());

    // Load c3d file and initialise frame offsetter
    C3DData c3d(c3dpath);
    FrameOffsetter fo(timeStamp, framerate, c3d.GetTimeCode(), c3d.GetFramerate() );

    // Using the c3d.timecode as reference, find which c3d frame corresponds to the provided timecodes.
    unsigned c3dFrameStart = CalculateFrameOffset(c3d.GetTimeCode(), startCode, subFramesStart);
    unsigned c3dFrameEnd = CalculateFrameOffset(c3d.GetTimeCode(), endCode, subFramesEnd);

    // Find out the video frame of the start and end timecodes
    unsigned vidFrameStart = fo.ToReference(c3dFrameStart);
    unsigned vidFrameEnd = fo.ToReference(c3dFrameEnd);

    // Calculate the timestamp at the new start frame
    unsigned long newTimestamp = AdvanceTimestampToFrame(timeStamp, framerate, vidFrameStart);

    std::cout << "------------" << std::endl;
    std::cout << "new start frame: " << vidFrameStart << std::endl;
    std::cout << "new   end frame: " << vidFrameEnd << std::endl;
    std::cout << "new timestamp  : " << newTimestamp << std::endl;

    return 0;
}
