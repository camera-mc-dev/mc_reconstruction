#include <vector>
#include <map>
#include <set>
#include <string>
#include <iostream>
#include <iomanip>
using std::vector;
using std::cout;
using std::endl;


#include "calib/calibration.h"

// #define OCCTRACK_DEBUG
#ifdef OCCTRACK_DEBUG
#include "renderer2/basicRenderer.h"
#endif

//
// In principle, this could be just a blob tracker - threshold a heatmap, do nearest neighbour associations
// between blob centres, and we don't need to make any reference to the fundamental geometry that lead
// to the creation of the occupancy map.
// 
// In practice, we probably want to be significantly more clever about how we do this. We also want to leave
// open the option for whether we do frame-to-frame tracking with an output every frame, or whether we do
// full sequence tracking and then give an output. Note that the priority is full-sequence tracking.
// 
// Another question we might have is regarding what we might want to do with multi-planar occupancy maps,
// and whether we want to deal with that internally, or ask the task-specific app to decide what to 
// do with those multiple tracking planes. I'm inclined to say that that is a task-specific problem and 
// thus we only have a single-plane occupancy map as input.
//
class OccupancyTracker
{
public:
	
	struct SOccTrackSettings
	{
		cv::Mat visMap;           // visibility map of the occupancy area.
		int minVisibility;        // minimum visibility of a peak.
		float detectionThreshold; // minimum occupancy.
		
		bool useVisibility;       // when computing occupancy is the computation count/cameras  or count/visibility?
		int numCameras;
		
		float distanceThreshold;  // this is based on the track distance, which is spatialDistance + log( timeDistance )
		int   numNearPeaks;       // When we create the initial tracks, we compute the distance vs. the numNearPeaks 
		                          // that come after it in space and time. A small number might mean you don't connect 
		                          // the track fully. A large number will mean a lot of redundant calculation.
	};
	
	struct SPeak
	{
		hVec2D mean;
		Eigen::Matrix2f cov;
		float confidence;
		float area;
	};
	
	struct STrack
	{
		std::map< int, SPeak > framePeaks;
		int startFrame;
		int endFrame;
		
		int merged; // only used when merging tracks.
	};
	
	OccupancyTracker(SOccTrackSettings inSettings);
	
	
	//
	// Given an occupancy map, add a new frame to our available tracking information.
	//
	cv::Mat AddFrame( int frameNo, cv::Mat occMap );
	
	//
	// Given the currently available frame data, get an optimal set of tracks.
	// 
	// Note that this starts from scratch every time it is called - it does not 
	// "update" a previous set of tracks, though that could be done in future.
	//
#ifdef OCCTRACK_DEBUG
	void GetTracks( std::vector< STrack > &tracks, std::shared_ptr<Rendering::BasicPauseRenderer> &dbgRen, cv::Mat &dbgImg );
#else
	void GetTracks( std::vector< STrack > &tracks );
#endif
	
protected:
	
	SOccTrackSettings settings;
	
	
	//
	// I don't want to store all the unprocessed occupancy maps, that would just be a bit too painful.
	// so, instead of that, we process a frame into a set of peaks where the peak is represented by a 
	// 2D Gaussian.
	//
	std::map< int, std::vector<SPeak> > frameDetections;
	
	void FitGaussian( cv::Mat img, int x0, int y0, int x1, int y1, hVec2D &mean, Eigen::Matrix2f &cov, float &confidence );
	
	
};
