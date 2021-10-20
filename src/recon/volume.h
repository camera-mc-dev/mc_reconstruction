#ifndef ME_VOLUME_H
#define ME_VOLUME_H

#include <vector>
using std::vector;

#include <thread>



#include "imgio/imagesource.h"
#include "math/mathTypes.h"
#include "misc/types.h"

// a VoxKey is an ID that tells us where in the octree a specific voxel is,
// and also helps us locate neighbours really quickly... hopefully!
struct VoxKey
{
	long int layer;
	long int x,y,z;
	
	bool operator<(const VoxKey& k) const
	{
		if( layer == k.layer )
		{
			if( z == k.z )
			{
				if( y == k.y )
				{
					return x < k.x;
				}
				else
				{
					return y < k.y;
				}
			}
			else
			{
				return z < k.z;
			}
		}
		else
		{
			return layer < k.layer;
		}
	}
	
// 	VoxKey operator=(const VoxKey &k)
// 	{
// 		layer = k.layer;
// 		x = k.x;
// 		y = k.y;
// 		z = k.z;
// 		return *this;
// 	}
	
	
	// octree is spatial, so each voxel has 6 neighbours
	void AllN( std::vector<VoxKey> &ns )
	{
		ns.clear(); ns.reserve(6);
		ns.push_back(RN());
		ns.push_back(LN());
		ns.push_back(FN());
		ns.push_back(BN());
		ns.push_back(AN());
		ns.push_back(UN());
	}
	VoxKey RN()    // right neighbour
	{
		VoxKey n;
		n.layer = layer;
		n.x = x + 1; n.y = y; n.z = z;
		return n;
	}
	VoxKey LN()    // left neighbour
	{
		VoxKey n;
		n.layer = layer;
		n.x = x - 1; n.y = y; n.z = z;
		return n;
	}
	VoxKey FN()    // front neighbour
	{
		VoxKey n;
		n.layer = layer;
		n.x = x; n.y = y + 1; n.z = z;
		return n;
	}
	VoxKey BN()    // back neighbour 
	{
		VoxKey n;
		n.layer = layer;
		n.x = x; n.y = y - 1; n.z = z;
		return n;
	}
	VoxKey AN()    // above neighbour
	{
		VoxKey n;
		n.layer = layer;
		n.x = x; n.y = y; n.z = z - 1;
		return n;
	}
	VoxKey UN()    // under neighbour
	{
		VoxKey n;
		n.layer = layer;
		n.x = x; n.y = y; n.z = z + 1;
		return n;
	}
	VoxKey P()     // parent
	{
		VoxKey p;
		p.layer = layer - 1;
		p.x = x / 2;
		p.y = y / 2;
		p.z = z / 2;
		return p;
	}
	
	
};


// A voxel is a 3D cube, defined by its centre and its side-lengths
struct Voxel
{
	// 3D information about the voxel
	hVec3D centre;
	float sideLength;
	
	// if we know where this cell is in the grid of the space,
	// then we know who its neighbours are. Now, we have so far 
	// managed to produce the oct-tree without explicitly storing it.
	// I think we can continue in that way by using the ID...
	VoxKey id;
	
	bool hasSplit;

	// which pixels does the voxel centre project to?
	// What are the extents of the voxel once projected into each image?
	vector< hVec2D > pixels;
	vector< Rect >   rects;
	vector<unsigned> visibility;
	vector<float>    satRatios;
};

typedef std::shared_ptr< Voxel > VoxPtr;

// Given n-images from n-calibrated cameras, the volume builder determines whether voxels
// in the working volume are occupied or not. The working volume is a cuboid of the provided
// xsize, ysize and zsize dimensions (presumably, metres). Each unit of size is divided into
// resolution voxels.
//
// Each voxel of the volume is projected into the source images. If the source image at that
// location is non-black, then the voxel is considered occupied.
//
// The volume builder returns a list of occupied voxels, which could, for example, be rendered,
// or used for a number of other tasks.
class VolumeBuilder
{
public:
	VolumeBuilder(float minX, float maxX, float minY, float maxY, float minZ, float maxZ,
	              float minVoxSize, float maxVoxSize, std::vector< ImageSource* > &imageSources);

	// A voxel is considered occupied when it projects to the foreground blob of minOccupation
	// viewpoints.
	// if minOccupation is set to 0 (default) then voxels are considered occupied if they project
	// to foreground in all viewpoints for which they are visible.
	// We input the foreground masks such that mask[0] is known to come from imageSources[0].
	void GetOccupiedVoxels(vector<cv::Mat> &masks, vector<VoxPtr> &occVoxels, float minOccupation=0);

private:

	// extents of the volume in each direction.
	// these directions are dependent upon the calibration of
	// the image sources.
	float minX, maxX;
	float minY, maxY;
	float minZ, maxZ;

	// voxel size in terms of calibration units (e.g. m or cm or mm etc...)
	// max is the largest and where voxels start before being split down to minimum size
	// as needed.
	float maxVoxSize, minVoxSize;

	// sources of image data. Input images are expected
	// to be binary foreground masks.
	std::vector< ImageSource* > imageSources;

	// all Voxels at the coarsest granularity of the space.
	std::vector< VoxPtr > rootVoxels;

	void ProjectVoxel(VoxPtr v);

	vector<cv::Mat> *masks;
	vector<cv::Mat> integralMasks;
	vector<cv::Mat> maskEdges;


	unsigned CheckVoxelOccupied(VoxPtr v, unsigned &saturated, unsigned &onEdges);
	bool SplitVoxel(VoxPtr v, std::vector< VoxPtr > &next );
	void ImgProcThread(unsigned start, unsigned end);
	void OccThread( unsigned start, unsigned end, vector<VoxPtr> &voxels, float minOccupation);
};

#endif
