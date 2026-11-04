#ifdef USE_EZC3D

#include "misc/c3d.h"

#include <ezc3d/ezc3d.h>
#include <ezc3d/Header.h>
#include <ezc3d/Data.h>

#include <iostream>
#include <cmath>
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

    auto names = c3d.pointNames();
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

    unsigned numFrames  = c3d.header().nbFrames();
    unsigned firstFrame = c3d.header().firstFrame();
    cout << "\t\t            nframes, fframe: " << numFrames << " " << firstFrame << endl;

    for( unsigned nc = 0; nc < names.size(); ++nc )
    {
        points[ names[nc] ] = genMatrix::Zero(6, numFrames + firstFrame);
    }

    unsigned analogSubframes = c3d.header().nbAnalogByFrame();
    unsigned numAnalogFrames = (numFrames + firstFrame) * analogSubframes;
    channels = genMatrix::Zero( cnames.size(), numAnalogFrames);

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
                points[ names[nc] ].col(fc + firstFrame) << p(0), p(1), p(2), p(3),  r, 1.0f;
            else
                points[ names[nc] ].col(fc + firstFrame) << p(0), p(1), p(2), p(3),  r, 0.0f;
        }

        for( unsigned nc = 0; nc < cnames.size(); ++nc )
        {
            for( unsigned sfc = 0; sfc < analogSubframes; ++sfc )
            {
                int idx = (firstFrame*analogSubframes) + (fc*analogSubframes) + sfc;
                channels(nc, idx) = c3d.data().frame(fc).analogs().subframe(sfc).channel(nc).data();
            }
        }
    }

    startFrame = firstFrame;
}

C3DData::C3DData(std::string filename)
{
    LoadFile(filename);
}

C3DData::~C3DData()
{
}

bool C3DData::LoadFile(std::string filename)
{
    LoadC3DFile(filename, startFrame, points, channels);
    numFrames = points.begin()->second.cols();
    return true;
}

bool C3DData::GetPointDataAtFrame(float frame, std::string pointName, hVec3D &point)
{
    if (points.find(pointName) == points.end())
    {
        return false;
    }

    int frame1 = static_cast<int>(frame);
    int frame2 = frame1 + 1;

    if (frame2 >= numFrames)
    {
        frame2 = numFrames - 1;
    }

    InterpolatePointData(frame, pointName, point);
    return true;
}

bool C3DData::GetAnalogDataAtFrame(float frame, std::string channelName, float &value)
{
    int channelIdx = -1;
    for (size_t i = 0; i < channels.rows(); ++i)
    {
        if (channels.row(i).name() == channelName)
        {
            channelIdx = i;
            break;
        }
    }

    if (channelIdx == -1)
    {
        return false;
    }

    int frame1 = static_cast<int>(frame);
    int frame2 = frame1 + 1;

    if (frame2 >= channels.cols())
    {
        frame2 = channels.cols() - 1;
    }

    InterpolateAnalogData(frame, channelName, value);
    return true;
}

void C3DData::InterpolatePointData(float frame, std::string pointName, hVec3D &point)
{
    int frame1 = static_cast<int>(frame);
    int frame2 = frame1 + 1;

    if (frame2 >= numFrames)
    {
        frame2 = numFrames - 1;
    }

    float alpha = frame - frame1;

    hVec3D p1 = points[pointName].col(frame1).head(3);
    hVec3D p2 = points[pointName].col(frame2).head(3);

    point = (1.0f - alpha) * p1 + alpha * p2;
}

void C3DData::InterpolateAnalogData(float frame, std::string channelName, float &value)
{
    int channelIdx = -1;
    for (size_t i = 0; i < channels.rows(); ++i)
    {
        if (channels.row(i).name() == channelName)
        {
            channelIdx = i;
            break;
        }
    }

    if (channelIdx == -1)
    {
        return;
    }

    int frame1 = static_cast<int>(frame);
    int frame2 = frame1 + 1;

    if (frame2 >= channels.cols())
    {
        frame2 = channels.cols() - 1;
    }

    float alpha = frame - frame1;

    float v1 = channels(channelIdx, frame1);
    float v2 = channels(channelIdx, frame2);

    value = (1.0f - alpha) * v1 + alpha * v2;
}

#endif
