#ifndef C3D_LOADER_H

#ifdef USE_EZC3D

#include <map>
#include <vector>
#include <string>
#include "math/mathTypes.h"

void LoadC3DFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points );
void LoadC3DFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points, genMatrix &channels );

class C3DData
{
public:
    C3DData(std::string filename);
    ~C3DData();

    bool LoadFile(std::string filename);
    bool GetPointDataAtFrame(float frame, std::string pointName, hVec3D &point);
    bool GetAnalogDataAtFrame(float frame, std::string channelName, float &value);

private:
    void InterpolatePointData(float frame, std::string pointName, hVec3D &point);
    void InterpolateAnalogData(float frame, std::string channelName, float &value);

    std::map<std::string, genMatrix> points;
    genMatrix channels;
    unsigned startFrame;
    unsigned numFrames;
};

#endif

#endif
