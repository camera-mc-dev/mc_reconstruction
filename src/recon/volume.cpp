#include "volume.h"

#include <iostream>
using std::cout;
using std::endl;

#include <deque>
using std::deque;

#include <map>

VolumeBuilder::VolumeBuilder(float minX, float maxX, float minY, float maxY, float minZ, float maxZ,
              float in_minVoxSize, float in_maxVoxSize, std::vector< ImageSource* > &imageSources)
	: minX(minX), maxX(maxX), minY(minY), maxY(maxY), minZ(minZ), maxZ(maxZ),
	  minVoxSize(in_minVoxSize), maxVoxSize(in_maxVoxSize), imageSources(imageSources)
{
	float x,y,z;
	
	rootVoxels.clear();
	std::cout << "Precomputing coarse level voxel projections... could take a wee moment..." << std::endl;
	unsigned count = 0;	// yeah, this is lazy, I could just work out how many we have..
	for( x = minX; x <= maxX; x += maxVoxSize )
	{
		for( y = minY; y <= maxY; y += maxVoxSize )
		{
			for( z = minZ; z < maxZ; z += maxVoxSize )
			{
				++count;
			}
		}
	}
	
	rootVoxels.resize(count);
	unsigned indx = 0;
	unsigned xi, yi, zi;
	xi = yi = zi = 0;
	for( x = minX; x <= maxX; x += maxVoxSize )
	{
		for( y = minY; y <= maxY; y += maxVoxSize )
		{
			for( z = minZ; z < maxZ; z += maxVoxSize )
			{
				VoxPtr v = VoxPtr( new Voxel );
				hVec3D vc; vc << x,y,z,1.0;
				v->centre = vc;
				v->sideLength = maxVoxSize;
				v->pixels.resize( imageSources.size() );
				ProjectVoxel(v);
				
				v->id.x = xi;
				v->id.y = yi;
				v->id.z = zi;
				v->id.layer = 0;
				
				rootVoxels[indx] = v;
				
				++indx;
				++zi;
				
				
			}
			++yi;
		}
		++xi;
	}
	std::cout << "done pre-projection" << endl;
}


void VolumeBuilder::ProjectVoxel(VoxPtr v)
{
	// all the voxel's corners.
	float shift = v->sideLength / 2.0f;
	vector<hVec3D> corns(8, v->centre);
	corns[0](0) -= shift; corns[0](1) += shift; corns[0](2) += shift;
	corns[1](0) += shift; corns[1](1) += shift; corns[1](2) += shift;
	corns[2](0) += shift; corns[2](1) -= shift; corns[2](2) += shift;
	corns[3](0) -= shift; corns[3](1) -= shift; corns[3](2) += shift;

	corns[4](0) -= shift; corns[4](1) += shift; corns[4](2) -= shift;
	corns[5](0) += shift; corns[5](1) += shift; corns[5](2) -= shift;
	corns[6](0) += shift; corns[6](1) -= shift; corns[6](2) -= shift;
	corns[7](0) -= shift; corns[7](1) -= shift; corns[7](2) -= shift;

	v->rects.clear();
	v->pixels.clear();
	for( unsigned isc = 0; isc < imageSources.size(); ++isc )
	{
		vector<hVec2D> pcorns(8);
		for( unsigned cc = 0; cc < 8; ++cc )
		{
			pcorns[cc] = imageSources[isc]->GetCalibration().Project(corns[cc]);
		}

		unsigned w, h;
		w = imageSources[isc]->GetCalibration().width;
		h = imageSources[isc]->GetCalibration().height;

		Rect r;
		r.topLeft << w, h, 1;
		r.botRight << 0, 0, 1;

		for( unsigned cc = 0; cc < 8; ++cc )
		{
			r.topLeft(0)  = std::min(r.topLeft(0) , pcorns[cc](0) );
			r.topLeft(1)  = std::min(r.topLeft(1) , pcorns[cc](1) );
			r.botRight(0) = std::max(r.botRight(0), pcorns[cc](0) );
			r.botRight(1) = std::max(r.botRight(1), pcorns[cc](1) );
		}

		// r.topLeft(0) -= 15;
		// r.topLeft(1) -= 15;
		//
		// r.botRight(0) += 15;
		// r.botRight(1) += 15;

		v->rects.push_back(r);
		v->pixels.push_back( imageSources[isc]->GetCalibration().Project( v->centre ) );

		if( r.topLeft(0) < w && r.topLeft(1) < h && r.botRight(0) > 0 && r.botRight(1) > 0)
			v->visibility.push_back(isc);
	}
}

unsigned VolumeBuilder::CheckVoxelOccupied(VoxPtr v, unsigned &saturated, unsigned &onEdges)
{
	// occupied tells us if the voxel at least partially overlaps
	// with the foreground regions.
	unsigned occupied = 0;

	// if all the pixels of the voxel's projection are deemed to
	// be foreground, then we call the voxel "saturated".
	// if it is saturated in all views then the voxel should not
	// be split into smaller components.
	saturated = 0;
	v->satRatios.clear();

	// simillarly, if a voxel is on the surface of the object,
	// then it should have some edge pixels inside of its
	// border. If it does not, then we probably don't
	// actually care about the voxel.
	onEdges = 0;

	// now, run through each view and determine
	// the level of saturation and edge count to
	// decide if occupied or not.
	std::vector<cv::Rect> rects(v->visibility.size());
	for( unsigned vc = 0; vc < v->visibility.size(); ++vc )
	{
		unsigned isc = v->visibility[vc];

		unsigned w, h;
		w = imageSources[isc]->GetCalibration().width;
		h = imageSources[isc]->GetCalibration().height;

		cv::Mat &i = integralMasks[isc];

		unsigned count = 0;
		unsigned xm = std::max(0.0f, v->rects[isc].topLeft(0));
		unsigned ym = std::max(0.0f, v->rects[isc].topLeft(1));

		unsigned xM = std::min((float)w-1.0f, v->rects[isc].botRight(0));
		unsigned yM = std::min((float)h-1.0f, v->rects[isc].botRight(1));

		unsigned visArea = (xM - xm) * (yM - ym);

		// masks are integral images, so getting the sum of foreground
		// pixels in the voxel's projection is now trivial...
		count  = i.at<int>( yM, xM );
		count += i.at<int>( ym, xm );
		count -= i.at<int>( yM, xm );
		count -= i.at<int>( ym, xM );

		rects[vc] = cv::Rect(xm, ym, xM-xm, yM-ym);

		float thresh = 0.000 * (float)visArea;
		if( (float)count > thresh )
		{
			++occupied;
		}
		if( count == visArea )
		{
			++saturated;
		}

		v->satRatios.push_back(count/ (float)visArea);
	}

	// checking if the voxel overlaps a mask edge is kind of slow,
	// but we don't always need to do it - if a voxel is not saturated
	// on a viewpoint then it must be on the edge of the mask region.
	if( saturated == v->visibility.size() )
	{
		for( unsigned vc = 0; vc < v->visibility.size(); ++vc )
		{
			double m,M;
			unsigned isc = v->visibility[vc];
			cv::minMaxIdx(maskEdges[isc](rects[vc]), &m, &M);
			if( M > 0 )
				++onEdges;
		}
	}
	else
	{
		onEdges = v->visibility.size() - saturated;
	}


	return occupied;
}


bool VolumeBuilder::SplitVoxel(VoxPtr v, std::vector< VoxPtr > &next )
{

	// check that the input voxel is not already small enough.
	bool split = v->sideLength > minVoxSize;
	
	
	if( split )
	{
		// make 8 new voxels from the 8 octants of v
		float nsl = v->sideLength / 2.0f;
		float shift = nsl / 2.0f;

		vector<hVec3D> newCents(8, v->centre);
		newCents[0](0) -= shift; newCents[0](1) -= shift; newCents[0](2) -= shift;
		newCents[1](0) += shift; newCents[1](1) -= shift; newCents[1](2) -= shift;
		newCents[2](0) -= shift; newCents[2](1) += shift; newCents[2](2) -= shift;
		newCents[3](0) += shift; newCents[3](1) += shift; newCents[3](2) -= shift;

		newCents[4](0) -= shift; newCents[4](1) -= shift; newCents[4](2) += shift;
		newCents[5](0) += shift; newCents[5](1) -= shift; newCents[5](2) += shift;
		newCents[6](0) -= shift; newCents[6](1) += shift; newCents[6](2) += shift;
		newCents[7](0) += shift; newCents[7](1) += shift; newCents[7](2) += shift;
		
		vector< VoxKey > newKeys(8);
		newKeys[0].x = (v->id.x * 2);       newKeys[0].y = (v->id.y * 2);       newKeys[0].z = (v->id.z * 2);
		newKeys[1].x = (v->id.x * 2) + 1;   newKeys[1].y = (v->id.y * 2);       newKeys[1].z = (v->id.z * 2);
		newKeys[2].x = (v->id.x * 2);       newKeys[2].y = (v->id.y * 2) + 1;   newKeys[2].z = (v->id.z * 2);
		newKeys[3].x = (v->id.x * 2) + 1;   newKeys[3].y = (v->id.y * 2) + 1;   newKeys[3].z = (v->id.z * 2);
		                                                                                           
		newKeys[4].x = (v->id.x * 2);       newKeys[4].y = (v->id.y * 2);       newKeys[4].z = (v->id.z * 2) + 1;
		newKeys[5].x = (v->id.x * 2) + 1;   newKeys[5].y = (v->id.y * 2);       newKeys[5].z = (v->id.z * 2) + 1;
		newKeys[6].x = (v->id.x * 2);       newKeys[6].y = (v->id.y * 2) + 1;   newKeys[6].z = (v->id.z * 2) + 1;
		newKeys[7].x = (v->id.x * 2) + 1;   newKeys[7].y = (v->id.y * 2) + 1;   newKeys[7].z = (v->id.z * 2) + 1;
		
		for( unsigned nvc = 0; nvc < 8; ++nvc )
		{
			newKeys[nvc].layer = v->id.layer + 1;
			
// 			cout << newKeys[nvc].layer << " " << newKeys[nvc].x << " " << newKeys[nvc].y << " " << newKeys[nvc].z << " " << endl; 
		}
// 		cout << "--" << endl;

		
		for( unsigned nvc = 0; nvc < 8; ++nvc )
		{
			VoxPtr nv( new Voxel );
			nv->sideLength = nsl;
			nv->centre = newCents[nvc];
			nv->id = newKeys[nvc];

			// make sure new voxel is inside work space.
			if( nv->centre(0) + nv->sideLength/2.0f > minX && nv->centre(0) - nv->sideLength/2.0f < maxX &&
				nv->centre(1) + nv->sideLength/2.0f > minY && nv->centre(1) - nv->sideLength/2.0f < maxY &&
				nv->centre(2) + nv->sideLength/2.0f > minZ && nv->centre(2) - nv->sideLength/2.0f < maxZ)
			{
				// project to images...
				nv->pixels.resize( imageSources.size() );
				ProjectVoxel(nv);

				next.push_back(nv);
			}
		}
	}
	
	return split;

}

void VolumeBuilder::ImgProcThread(unsigned start, unsigned end)
{
	for( unsigned mc = start; mc < end; ++mc )
	{
		cv::Mat gx,gy;
		cv::Sobel(masks->at(mc), gx, CV_16U, 1, 0);
		cv::Sobel(masks->at(mc), gy, CV_16U, 0, 1);
		maskEdges[mc] = 0.5 * gx + 0.5 * gy;
		integralMasks[mc] = cv::Mat( masks->at(mc).rows, masks->at(mc).cols, CV_32S, cv::Scalar(0) );
		for( unsigned y = 0; y < masks->at(mc).rows; ++y )
		{
			for( unsigned x = 0; x < masks->at(mc).cols; ++x )
			{
				unsigned isFG = 0;
				cv::Mat &i = masks->at(mc);
				switch( i.depth() )
				{
					case CV_8U:
						if( 130 < i.at<unsigned char>(y,x) )
							++isFG;
						break;
					case CV_8S:
						if( 64 < i.at<char>(y,x) )
							++isFG;
						break;
					case CV_32F:
						if( 0.5 < i.at<float>(y,x) )
							++isFG;
						break;
				}

				unsigned sum = isFG;

				if( x > 0 )
					sum += integralMasks[mc].at<int>(y,x-1);
				if( y > 0 )
					sum += integralMasks[mc].at<int>(y-1,x);
				if( x > 0 && y > 0 )
					sum -= integralMasks[mc].at<int>(y-1,x-1);

				integralMasks[mc].at<int>(y,x) = sum;
			}
		}
	}
}

void VolumeBuilder::GetOccupiedVoxels(vector<cv::Mat> &in_masks, vector<VoxPtr> &voxels, float minOccupation)
{
	voxels.clear();

	// get integral images of the masks.
	auto intStart = std::chrono::steady_clock::now();
	masks = &in_masks;
	integralMasks.resize(masks->size() );
	maskEdges.resize(masks->size());
	unsigned numThreads = std::min( masks->size(), (size_t)4 );
	size_t ni = ceil( masks->size() / (float)numThreads);
	size_t s = 0;
	vector<std::thread> intThreads;
	for( unsigned tc = 0; tc < numThreads; ++tc )
	{
		unsigned e = std::min( masks->size(), s + ni);
		intThreads.push_back( std::thread(&VolumeBuilder::ImgProcThread, this, s, e ) );
		s = e;
	}
	for( unsigned tc = 0; tc < numThreads; ++tc )
		intThreads[tc].join();
	intThreads.clear();
	auto intEnd = std::chrono::steady_clock::now();




	auto voxStart = std::chrono::steady_clock::now();
	
	
	std::map< VoxKey, bool > voxOcc;
	std::vector<VoxPtr> work, next;
	work.insert( work.end(), rootVoxels.begin(), rootVoxels.end() );
	bool done = false;
	while( !done )
	{
		next.clear();
		next.reserve( work.size() * 8 );	// worst case is all voxels split.
		// process the work set.
		
		std::vector<bool> split; split.assign( work.size(), false );
		std::vector<unsigned> occupied; occupied.assign( work.size(), 0 );
		//#pragma omp parallel for
		for( unsigned vc = 0; vc < work.size(); ++vc )
		{
			VoxPtr v = work[vc];
			unsigned saturated = 0;
			unsigned onEdges = 0;
			occupied[vc] = CheckVoxelOccupied(v, saturated, onEdges);
			
			// options for visibility
			// 1) minOccupation > 1: at least this many cameras must have foreground for this voxel.
			// 2) 0 < minOccupation <= 1.0 : occupied/visibility > minOccupation
			
			if(    
			     ( (minOccupation <= 1.1 ) && (v->visibility.size() > 0) && ((float)occupied[vc]/v->visibility.size() > minOccupation ) ) ||
			     ( (minOccupation > 1 )    && occupied[vc] >= minOccupation)
			  )
// 			if( v->visibility.size() > minOccupation )
			{
				voxOcc[ v->id ] = true;
			}
		}
		for( unsigned vc = 0; vc < work.size(); ++vc )
		{
			VoxPtr v = work[vc];
			if( voxOcc[ v->id ] )
			{
				split[vc] = SplitVoxel( v, next );
			}
		}
		
		// the voxels that did not split but which are occupied can go
		// on our final voxel list. However, we don't want voxels on our 
		// final list that are not on the skin of the object. Voxels that 
		// are not on the skin will not have any unoccupied neighbours.
		std::vector<bool> keep;
		keep.assign( work.size(), false );
		#pragma omp parallel for
		for( unsigned vc = 0; vc < work.size(); ++vc )
		{
			VoxPtr v = work[vc];
			if( voxOcc[ v->id ] && !split[vc] )
			{
				std::vector< VoxKey > ns;
				v->id.AllN( ns );
				
// 				cout << v->id.layer << " " << v->id.x << " " << v->id.y << " " << v->id.z << endl;
// 				cout << "-->" << endl;
				
				unsigned occCount = 0;
				for( unsigned nc = 0; nc < ns.size(); ++nc )
				{
// 					cout << "\t" << ns[nc].layer << " " << ns[nc].x << " " << ns[nc].y << " " << ns[nc].z << endl;
					auto vn = voxOcc.find( ns[nc] );
					if( vn != voxOcc.end() )
					{
						if( vn->second )
							++occCount;
					}
				}
				if( occCount != ns.size() )
				{
					keep[vc] = true;
				}
			}
		}
		
		for( unsigned vc = 0; vc < work.size(); ++vc )
		{
			if( keep[vc] )
				voxels.push_back( work[vc] );
		}
		
		
		// now next becomes work, and we carry on,
		// unless there's nothing to work on of course.
		work.clear();
		work.resize( next.size() );
		for( unsigned vc = 0; vc < next.size(); ++vc )
			work[vc] = next[vc];
		if( work.size() == 0 )
			done = true;
		
// 		cout << "--- work ---" << endl;
// 		for( unsigned vc = 0; vc < work.size(); ++vc )
// 		{
// 			cout << work[vc]->id.layer << " " << work[vc]->id.x << " " << work[vc]->id.y << " " << work[vc]->id.z << " " << endl; 
// 		}
// 		cout << "--" << endl;
		
		
		// NOTE:
		// If we checked for skin voxels by looking at next, we might be able to
		// avoid splitting all occupied voxels down to full resolution, which would 
		// be a significant performance boost. However, to do so would mean being 
		// more clever in the check for occupied neighbours, as we would then also
		// need to check voxOcc for previous layers.
	}
	

	auto voxEnd = std::chrono::steady_clock::now();

	cout << "\tint: " << std::chrono::duration <double, std::milli>(intEnd - intStart).count() << endl;
	cout << "\tvox: " << std::chrono::duration <double, std::milli>(voxEnd - voxStart).count() << endl;




}
