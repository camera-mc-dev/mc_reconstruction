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

class OccupancyMap
{
public:
	enum upDir_t {UP_X, UP_Y, UP_Z};
	enum proj_t  {PROJ_LINE, PROJ_BB};
	
	struct SObsPlane
	{
		float low;
		float high;
	};
	
	struct SOccMapSettings
	{
		//
		// Limits of the observation area
		//
		float minX;
		float maxX;
		float minY;
		float maxY;
		float minZ;
		float maxZ;
		
		float cellSize;
		
		//
		// What direction is "up"
		//
		upDir_t upDir;
		
		
		//
		// Observation planes (height ranges)
		//
		std::vector< SObsPlane > obsPlanes;
		
		
		//
		// calibrations
		//
		std::vector< Calibration > calibs;
		
		//
		// enlarge a cell before projection?
		// (means cells can overlap with other cells)
		//
		float cellPadding;
		
	};
	
	//
	// Constructor. 
	//
	OccupancyMap( SOccMapSettings settings );
	
	
	//
	// Map makers
	//
	
	// create an occupancy map by projecting cells into the masks
	// and summing along the projection line, or within the projection bb.
	// masks should be CV_8UC1 or CV_32FC1
	// maps will be CV_32FC1
	void OccupancyFromSegmentation( 
	                                std::vector< cv::Mat > &masks,
	                                proj_t projectionType, 
	                                std::vector< cv::Mat > &maps
	                              );
	
	// create an occupancy map by projecting cells into an image and 
	// checking if the input points are within the cell.
	// projection type is assumed to be bbox. 
	void OccupancyFromPoints( 
	                           std::vector< std::vector< hVec2D > > &points, 
	                           std::vector< cv::Mat > &maps
	                        );
	
	// create an occupancy map by projecting cells into an image and 
	// checking the overlap of projection and the provided bboxes.
	// projection type is assumed to be bbox. 
	void OccupancyFromBBoxes( 
	                           std::vector< std::vector< cv::Rect > > &bboxes, 
	                           std::vector< cv::Mat > &maps
	                        );
	
	void GetLineVisibility( std::vector<cv::Mat> &visMaps );
	void GetBBVisibility( std::vector<cv::Mat> &visMaps );
	
	
	
	//
	// This assumes you have already pre-processed the occupancy in the way _you_ want with regards to 
	// visibility and other normalisations.
	//
	void FindPeaks( 
	                cv::Mat &occ,
	                std::vector< hVec3D > oldPeaks,
	                int regionInd,
	                float exclusionRadius,
	                float minPeakValue,
	                std::vector< hVec3D > &points,
	                std::vector< Eigen::Vector3f > &peaks
	              );
	
	hVec3D CellAsPoint( unsigned plane, unsigned row, unsigned col )
	{
		return groundPoints[row][col] + upDir * (settings.obsPlanes[plane].low + settings.obsPlanes[plane].high)/2.0f;
	}
	
	cv::Rect GetCellBBox( unsigned plane, unsigned row, unsigned col, unsigned view )
	{
		return bboxes[row][col][plane][view];
	}
	
	
	int GetMapRows() {return mapRows;}
	int GetMapCols() {return mapCols;}
	
protected:
	
	SOccMapSettings settings;
	
	
	//
	// Pre-computations
	//
	void PrecomputeBBProjections();
	void PrecomputeLineProjections();
	
	// these will be [bc][ac][plane][view]
	std::vector< std::vector< std::vector< std::vector<  cv::Rect > > > > bboxes;
	std::vector< std::vector< std::vector< std::vector< genMatrix > > > > lines;
	
	
	//
	// Utility methods
	//
	void ComputeIntegralImages( std::vector< cv::Mat > &masks );
	std::vector< cv::Mat > intMasks;
	float GetFGRatio( cv::Mat intMask, cv::Rect bb );
	float GetFGRatio( cv::Mat mask, genMatrix linePoints );
	float CheckPoints( std::vector< hVec2D > &points, cv::Rect bb );
	
	//
	// Other data
	//
	std::vector<cv::Mat> lineVisibility;
	std::vector<cv::Mat> bboxVisibility;
	
	std::vector< std::vector< hVec3D > > groundPoints;
	hVec3D upDir;
	hVec3D aDir;
	hVec3D bDir;
	
	unsigned mapRows, mapCols;
	
	
};
