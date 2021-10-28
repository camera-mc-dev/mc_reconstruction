#include "recon/occupancy.h"

#include "renderer2/showImage.h"

OccupancyMap::OccupancyMap( SOccMapSettings in_settings ) : settings( in_settings )
{
	//
	// Let's start by computing the grid points on the "ground",
	// which is to say, height = 0
	//
	int numCellsA, numCellsB;
	float minA, maxA;
	float minB, maxB;
	float minH, maxH;
	switch( settings.upDir )
	{
		case UP_X:
			minA = settings.minY;
			maxA = settings.maxY;
			minB = settings.minZ;
			maxB = settings.maxZ;
			aDir  << 0,1,0,0;
			bDir  << 0,0,1,0;
			upDir << 1,0,0,0;
			break;
			
		case UP_Y:
			minA = settings.minX;
			maxA = settings.maxX;
			minB = settings.minZ;
			maxB = settings.maxZ;
			aDir  << 1,0,0,0;
			bDir  << 0,0,1,0;
			upDir << 0,1,0,0;
			break;
			
		case UP_Z:
			minA = settings.minX;
			maxA = settings.maxX;
			minB = settings.minY;
			maxB = settings.maxY;
			aDir  << 1,0,0,0;
			bDir  << 0,1,0,0;
			upDir << 0,0,1,0;
			break;
	}
	
	numCellsA = (maxA-minA) / settings.cellSize;
	numCellsB = (maxB-minB) / settings.cellSize;
	mapRows = numCellsB;
	mapCols = numCellsA;
	
	groundPoints.resize( numCellsB );
	for( unsigned bc = 0; bc < numCellsB; ++bc )
	{
		groundPoints[bc].resize( numCellsA );
		for( unsigned ac = 0; ac < numCellsA; ++ac )
		{
			switch( settings.upDir )
			{
				case UP_X:
					groundPoints[bc][ac] << 0.0f,
					                        minA + (ac+0.5) * settings.cellSize,
					                        minB + (bc+0.5) * settings.cellSize,
					                        1.0f;
					break;
				case UP_Y:
					groundPoints[bc][ac] << minA + (ac+0.5) * settings.cellSize,
					                        0.0f,
					                        minB + (bc+0.5) * settings.cellSize,
					                        1.0f;
					break;
					
				case UP_Z:
					groundPoints[bc][ac] << minA + (ac+0.5) * settings.cellSize,
					                        minB + (bc+0.5) * settings.cellSize,
					                        0.0f,
					                        1.0f;
					break;
			}
		}
	}
	
	cout << "map rows/cols: " << mapRows << " " << mapCols << endl;
	
	PrecomputeBBProjections();
	PrecomputeLineProjections();
}


void OccupancyMap::OccupancyFromSegmentation( 
                                              std::vector< cv::Mat > &masks,
                                              proj_t projectionType, 
                                              std::vector< cv::Mat > &maps
                                            )
{
	if( projectionType == PROJ_BB )
		ComputeIntegralImages( masks );
	maps.resize( settings.obsPlanes.size() );
	for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
	{
		maps[pc] = cv::Mat( mapRows, mapCols, CV_32FC1, cv::Scalar(0.0f) );
	}
	for( unsigned bc = 0; bc < mapRows; ++bc )
	{
		for( unsigned ac = 0; ac < mapCols; ++ac )
		{
// 			#pragma omp parallel for
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
			{
				for( unsigned vc = 0; vc < settings.calibs.size(); ++vc )
				{
					if( projectionType == PROJ_BB )
					{
						maps[pc].at<float>(bc,ac) += GetFGRatio( intMasks[vc], bboxes[bc][ac][pc][vc] );
					}
					else if( projectionType == PROJ_LINE )
					{
						maps[pc].at<float>(bc,ac) += GetFGRatio( masks[vc], lines[bc][ac][pc][vc] );
					}
				}
			}
		}
	}
}


float OccupancyMap::CheckPoints( std::vector< hVec2D > &points, cv::Rect bb )
{
	int in = 0;
	for( unsigned pc = 0; pc < points.size(); ++pc )
	{
		if(
		    points[pc](0) >= bb.x && points[pc](0) < bb.x + bb.width &&
		    points[pc](1) >= bb.y && points[pc](1) < bb.y + bb.height 
		  )
		{
			++in;
		}
	}
	return std::min(1,in);
}


void OccupancyMap::OccupancyFromPoints( 
                                         std::vector< std::vector< hVec2D > > &points, 
                                         std::vector< cv::Mat > &maps
                                      )
{
	// put simply, is the point inside the bbox of the cell?
	maps.resize( settings.obsPlanes.size() );
	for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
	{
		maps[pc] = cv::Mat( mapRows, mapCols, CV_32FC1, cv::Scalar(0.0f) );
	}
	for( unsigned bc = 0; bc < mapRows; ++bc )
	{
		for( unsigned ac = 0; ac < mapCols; ++ac )
		{
// 			#pragma omp parallel for
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
			{
				for( unsigned vc = 0; vc < settings.calibs.size(); ++vc )
				{
					maps[pc].at<float>(bc,ac) += CheckPoints( points[vc], bboxes[bc][ac][pc][vc] );
				}
			}
		}
	}
}


void OccupancyMap::GetLineVisibility( std::vector<cv::Mat> &visMaps )
{
	visMaps = lineVisibility;
}



void OccupancyMap::GetBBVisibility( std::vector<cv::Mat> &visMaps )
{
	visMaps = bboxVisibility;
}




void OccupancyMap::PrecomputeBBProjections()
{
	cout << "pre-comp bb proj" << endl;
	bboxes.resize( groundPoints.size() );
	bboxVisibility.resize( settings.obsPlanes.size() );
	for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
	{
		bboxVisibility[pc] = cv::Mat( mapRows, mapCols, CV_8UC1, cv::Scalar(0) );
	}
	for( unsigned bc = 0; bc < groundPoints.size(); ++bc )
	{
		bboxes[bc].resize( groundPoints[bc].size() );
		for( unsigned ac = 0; ac < bboxes[bc].size(); ++ac )
		{
			bboxes[bc][ac].resize( settings.obsPlanes.size() );
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
				bboxes[bc][ac][pc].resize( settings.calibs.size() );
			#pragma omp parallel for
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
			{
				hVec3D low  = groundPoints[bc][ac] + upDir * settings.obsPlanes[pc].low;
				hVec3D high = groundPoints[bc][ac] + upDir * settings.obsPlanes[pc].high;
				
				std::vector<hVec3D> pts(8);
				float d = settings.cellSize/2 + settings.cellPadding;
				pts[0] = low - d*aDir - d*bDir;
				pts[1] = low - d*aDir + d*bDir;
				pts[2] = low + d*aDir + d*bDir;
				pts[3] = low + d*aDir - d*bDir;
				
				pts[4] = high - d*aDir - d*bDir;
				pts[5] = high - d*aDir + d*bDir;
				pts[6] = high + d*aDir + d*bDir;
				pts[7] = high + d*aDir - d*bDir;
				
				for( unsigned vc = 0; vc < settings.calibs.size(); ++vc )
				{
					std::vector<hVec2D> projections(8);
					float minU, minV;
					float maxU, maxV;
					minU = settings.calibs[pc].width-1;
					minV = settings.calibs[pc].height-1;
					maxU = maxV = 0;
					for( unsigned ptc = 0; ptc < pts.size(); ++ptc )
					{
						projections[ptc] = settings.calibs[vc].Project( pts[ptc] );
						minU = std::min( projections[ptc](0), minU );
						maxU = std::max( projections[ptc](0), maxU );
						
						minV = std::min( projections[ptc](1), minV );
						maxV = std::max( projections[ptc](1), maxV );
					}
					
					if( maxU > minU && maxV > minV )
					{
						// we should be visible, right?
						bboxes[bc][ac][pc][vc] = cv::Rect( minU, minV, maxU-minU, maxV-minV );
						
						bboxVisibility[pc].at<unsigned char>(bc,ac) += 1;
					}
					else
					{
						bboxes[bc][ac][pc][vc] = cv::Rect( -1, -1, 0, 0 );
					}
					
					
				}
			}
		}
	}
}


void OccupancyMap::PrecomputeLineProjections()
{
	cout << "pre-comp line projections" << endl;
	lines.resize( groundPoints.size() );
	lineVisibility.resize( settings.obsPlanes.size() );
	std::vector< std::vector< hVec2D > > bufs( settings.obsPlanes.size() );
	std::vector< int > numPts( settings.obsPlanes.size() );
	for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
	{
		lineVisibility[pc] = cv::Mat( mapRows, mapCols, CV_32FC1, cv::Scalar(0) );
		bufs[pc].resize( settings.calibs[pc].height * 1.25 );
		numPts[pc] = 0;
	}
	for( unsigned bc = 0; bc < groundPoints.size(); ++bc )
	{
		lines[bc].resize( groundPoints[bc].size() );
		for( unsigned ac = 0; ac < lines[bc].size(); ++ac )
		{
			lines[bc][ac].resize( settings.obsPlanes.size() );
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
				lines[bc][ac][pc].resize( settings.calibs.size() );
// 			#pragma omp parallel for
			for( unsigned pc = 0; pc < settings.obsPlanes.size(); ++pc )
			{
				hVec3D low  = groundPoints[bc][ac] + upDir * settings.obsPlanes[pc].low;
				hVec3D high = groundPoints[bc][ac] + upDir * settings.obsPlanes[pc].high;
				
				
				
				for( unsigned vc = 0; vc < settings.calibs.size(); ++vc )
				{
					hVec2D plow, phigh;
					plow = settings.calibs[vc].Project( low );
					phigh = settings.calibs[vc].Project( high );
					
					
					if( 
						(
						  plow(0) >= 0 &&
						  plow(1) >= 0 &&
						  plow(0) < settings.calibs[vc].width &&
						  plow(1) < settings.calibs[vc].height  
						)
						||
						(
						  phigh(0) >= 0 &&
						  phigh(1) >= 0 &&
						  phigh(0) < settings.calibs[vc].width &&
						  phigh(1) < settings.calibs[vc].height  
						)
					  )
					{
						hVec2D d = plow - phigh;
						
						
						float dn = d.norm();
						d /= dn;
						
						numPts[pc] = 0;
						for( int c = 0; c < dn; c += 1.0f )
						{
							hVec2D x = phigh + c * d;
							if( x(0) >= 0 &&
								x(1) >= 0 &&
								x(0) < settings.calibs[vc].width &&
								x(1) < settings.calibs[vc].height  )
							{
								bufs[pc][numPts[pc]] = x;
								++numPts[pc];
								
							}
							
						}
						
						lines[bc][ac][pc][vc] = genMatrix::Zero(3, numPts[pc] );
						for( unsigned gc = 0; gc < numPts[pc]; ++gc )
						{
							lines[bc][ac][pc][vc].col(gc) = bufs[pc][gc];
						}
						
						
						// set the visibility.
						lineVisibility[pc].at<float>(bc,ac) += (float)numPts[pc] / dn;
					} // if plow or phigh project into the image
				} // for vc
			} // for pc
		} // for ac 
	} // for bc
}



void OccupancyMap::ComputeIntegralImages(std::vector< cv::Mat > &masks)
{
	throw std::runtime_error("Not done integral images yet. Don't score cell by bbox area.");
}

float OccupancyMap::GetFGRatio( cv::Mat intMask, cv::Rect bb )
{
	return 0.0f;
}


float OccupancyMap::GetFGRatio( cv::Mat mask, genMatrix linePoints )
{
	float res = 0.0f;
	for( unsigned pc = 0; pc < linePoints.cols(); ++pc )
	{
		if( mask.type() == CV_8UC1 )
		{
			res += mask.at<unsigned char>( linePoints(1,pc), linePoints(0,pc) ) / 255.0f;
		}
		else
		{
			res += mask.at<float>( linePoints(1,pc), linePoints(0,pc) );
		}
	}
	return res /= std::max(1.0f, (float)linePoints.cols() );
}













void OccupancyMap::FindPeaks( 
	                          cv::Mat &occ,
	                          std::vector< hVec3D > oldPeaks,
	                          int regionInd,
	                          float exclusionRadius,
	                          float minPeakValue,
	                          std::vector< hVec3D > &points,
	                          std::vector< Eigen::Vector3f > &peaks
	                        )
{
	// what is the exclusion ratio in terms of cells?
	int exCells = exclusionRadius / settings.cellSize;
	
	//
	// TODO: Handle using the previous peaks.
	//
	double m, M;
	cv::Point mx, Mx;
	
	
	//
	// Find our peaks.
	//
	cv::Mat occCpy = occ.clone();
	do
	{
		cv::minMaxLoc(occCpy, &m, &M, &mx, &Mx );
		
		if( M > minPeakValue )
		{
			Eigen::Vector3f p;
			p << Mx.x, Mx.y, M;
			peaks.push_back( p );
			
			cv::circle( occCpy, Mx, exCells, cv::Scalar(0.0f), -1 );
		}
	}
	while(M > minPeakValue );
	
	//
	// Convert peaks to points.
	//
	for( unsigned pc = 0; pc < peaks.size(); ++pc )
	{
		hVec3D p = groundPoints[peaks[pc](1)][peaks[pc](0)] + upDir * (settings.obsPlanes[regionInd].low + settings.obsPlanes[regionInd].high)/2.0f;
		points.push_back(p);
	}
	
}




