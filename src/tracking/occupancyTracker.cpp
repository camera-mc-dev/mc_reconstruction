#include "occupancyTracker.h"

#include "gnnTracker/tracker.h"


#include <fstream>

#include <chrono>
#include <thread>


OccupancyTracker::OccupancyTracker(OccupancyTracker::SOccTrackSettings inSettings)
{
	settings = inSettings;
}



cv::Mat OccupancyTracker::AddFrame( int frameNo, cv::Mat occMap )
{
	cv::Mat dbg( occMap.rows, occMap.cols, CV_32FC3, cv::Scalar(0,0,0) );
	
	//
	// First thing we're going to do is normalise the occupancy map
	// using either the per-cell visibility or just the number of 
	// views.
	//
	if( settings.useVisibility )
	{
		assert( settings.visMap.rows == occMap.rows );
		assert( settings.visMap.cols == occMap.cols );
		
		if( settings.visMap.type() == CV_8UC1 )
		{
			#pragma omp parallel for
			for( unsigned rc = 0; rc < occMap.rows; ++rc )
			{
				for( unsigned cc = 0; cc < occMap.cols; ++cc )
				{
					occMap.at<float>(rc,cc) /= settings.visMap.at<unsigned char>(rc,cc);
				}
			}
		}
		else if( settings.visMap.type() == CV_32FC1 )
		{
			#pragma omp parallel for
			for( unsigned rc = 0; rc < occMap.rows; ++rc )
			{
				for( unsigned cc = 0; cc < occMap.cols; ++cc )
				{
					occMap.at<float>(rc,cc) /= settings.visMap.at<float>(rc,cc);
				}
			}
		}
		
	}
	else
	{
		occMap /= (float)settings.numCameras;
	}
	
	//
	// Next we want to just get rid of anything lower than our detection threshold, or which 
	// has too-low visibility.
	//
	if( settings.visMap.type() == CV_8UC1 )
	{
		#pragma omp parallel for
		for( unsigned rc = 0; rc < occMap.rows; ++rc )
		{
			for( unsigned cc = 0; cc < occMap.cols; ++cc )
			{
				float &o = occMap.at<float>(rc,cc);
				unsigned char &v = settings.visMap.at<unsigned char>(rc,cc);
				
				if( o < settings.detectionThreshold || v < settings.minVisibility )
				{
					o = 0;
				}
			}
		}
	}
	else if( settings.visMap.type() == CV_32FC1 )
	{
		#pragma omp parallel for
		for( unsigned rc = 0; rc < occMap.rows; ++rc )
		{
			for( unsigned cc = 0; cc < occMap.cols; ++cc )
			{
				float &o = occMap.at<float>(rc,cc);
				float &v = settings.visMap.at<float>(rc,cc);
				
				if( o < settings.detectionThreshold || v < settings.minVisibility )
				{
					o = 0;
				}
			}
		}
	}
	
	
	//
	// Right, the occupancy mapper already had a way of detecting peaks, but we're going to 
	// ignore that a little bit to try and gather a bit more information - specifically, the 
	// spread of a peak, giving us some more lee-way on merging and splitting.
	//
	// But how should we handle a large blob with multiple peaks in it? 
	//
	// For now, my inclination is to just treat each blob as a detection and worry not about 
	// there being multiple peaks in that blob.
	//
	
	// so with that in mind, all we need is to threshold the map and do connected components.
	cv::Mat mt, mt32;
	cv::threshold(occMap, mt32, 0.01, 255, cv::THRESH_BINARY );
	mt32.convertTo( mt, CV_8UC1 );
	cv::Mat labels, stats, centroids;
	cv::connectedComponentsWithStats(mt, labels, stats, centroids);
	
	// now for each component we go through the set of points
	// and we want to get a 2D Gaussian that kind of represents the blob. 
	// Now, being really clever, we want to take into account the occupancy
	// of each pixel too.
	for( unsigned lc = 1; lc < stats.rows; ++lc )
	{
		SPeak peak;
		FitGaussian( occMap, 
		             stats.at<int>(lc, cv::CC_STAT_LEFT),
		             stats.at<int>(lc, cv::CC_STAT_TOP),
		             stats.at<int>(lc, cv::CC_STAT_LEFT) + stats.at<int>(lc, cv::CC_STAT_WIDTH),
		             stats.at<int>(lc, cv::CC_STAT_TOP)  + stats.at<int>(lc, cv::CC_STAT_HEIGHT),
		             peak.mean,
		             peak.cov,
		             peak.confidence
		           );
		peak.area = stats.at<int>(lc, cv::CC_STAT_AREA);
		frameDetections[frameNo].push_back( peak );
		//cv::circle( dbg, cv::Point( peak.mean(0), peak.mean(1) ), 4, cv::Scalar(0,0,1.0) );
		
		cout << lc << ": " << peak.mean.transpose() << " : " << peak.confidence << " " << peak.area << endl;
	}
	
	
	for( unsigned pc = 0; pc < frameDetections[frameNo].size(); ++pc )
	{
		SPeak &peak = frameDetections[frameNo][pc];
		#pragma omp parallel for
		for( unsigned rc = 0; rc < occMap.rows; ++rc )
		{
			for( unsigned cc = 0; cc < occMap.cols; ++cc )
			{
				cv::Vec3f &p = dbg.at< cv::Vec3f >(rc,cc);
				float &v = p[2];
				
				// given a 2D Gaussian with mean and cov what is the value at (cc,rc)?
				hVec2D d; d << cc,rc,1.0f;
				d = d - peak.mean;
				float top = exp( -0.5 * d.head(2).transpose() * peak.cov.inverse() * d.head(2) );
				//float bot = sqrt( (2*3.1415*peak.cov).norm() ); // don't care about this for vis.
				
				p[1] = std::max( p[1], top * peak.confidence ) ;
				p[2] = occMap.at<float>(rc,cc);
			}
		}
	}
	
	
	return dbg;
}


void DrawEllipse( cv::Mat &img, OccupancyTracker::SPeak &peak, cv::Scalar( color ), int thickness )
{
	float a = peak.cov(0,0);
	float b = peak.cov(1,0);
	float c = peak.cov(1,1);
	
	float t0 = (a+c)/2;
	float t1 = (a-c)/2;
	float t2 = sqrt( t1*t1 + b*b );
	
	float l1 = t0 + t2;
	float l2 = t0 - t2;
	
	float th;
	if( b == 0 && a >= c )
		th = 0;
	else if( b == 0 && a < c  )
		th = 3.14159 / 2.0f;
	else 
		th = atan2(l1-a,b);
	
	
	cv::Size axes(l1,l2);
	cv::Point centre( peak.mean(0), peak.mean(1) );
	cv::ellipse( img, centre, axes, th, 0, 360, color, thickness );
}



void UpdateTracks( int fc, std::vector<OccupancyTracker::STrack> &tracks, std::vector< SPeak > &dets )
{
	// I'm not going to do hard associations of detections to tracks.
	// Instead, I'm going to go _mad_ and set each track to a weighted mean of 
	// _all_ detections. The key, obviously, is setting the weights, and we set the
	// weights based on the distance of the detection to the track's previous position.
	
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		int tfc;
		if( fc > tracks[tc].endFrame )
			tfc = tracks[tc].endFrame;
		else if( fc < tracks[tc].startFrame )
			tfc = tracks[tc].startFrame;
		else 
			throw std::runtime_error("expecting new frames to be outside existing track time");
		
		std::vector<float> dists( dets.size() );
		for( unsigned dc = 0; dc < dets.size(); ++dc )
		{
			dists[dc] = (tracks[tc].framePeaks[tfc].mean - dets[dc].mean).norm();
		}
		
		// use a softmin to set the weights.
		// softmin = softmax(-x)
		std::vector<float> weights( dists.size() );
		float sum = 0.0f;
		for( unsigned wc = 0; wc < weights.size(); ++wc )
		{
			weights[wc] = exp( -dists[wc] );
			sum += weights[wc];
		}
		for( unsigned wc = 0; wc < weights.size(); ++wc )
			weights[wc] /= sum;
		
		hVec2D mean; mean << 0,0,0;
		Eigen::Matrix2f cov; cov << 0,0,   0,0;
		float conf = 0;
		float area = 0;
		for( unsigned wc = 0; wc < weights.size(); ++wc )
		{
			mean += weights[wc] * dets[wc].mean;
			cov  += weights[wc] * dets[wc].cov;  // I'm going to pretend that's true because it doesn't really matter
			conf += weights[wc] * dets[wc].conf;
			area += weights[wc] * dets[wc].area;
		}
		mean /= mean(2);
		cov  /= mean(2);
		conf /= mean(2);
		area /= mean(2);
		
		tracks[tc].framePeaks[fc].mean       = mean;
		tracks[tc].framePeaks[fc].cov        = cov;
		tracks[tc].framePeaks[fc].confidence = conf;
		tracks[tc].framePeaks[fc].area       = area;
		
		tracks[tc].startFrame = std::min( tracks[tc].startFrame, fc );
		tracks[tc].endFrame = std::max( tracks[tc].endFrame, fc );
	}
}



#ifdef OCCTRACK_DEBUG
void OccupancyTracker::GetTracks( std::vector< OccupancyTracker::STrack > &tracks, std::shared_ptr<Rendering::BasicPauseRenderer> &dbgRen, cv::Mat &dbgImg )
#else
void OccupancyTracker::GetTracks( std::vector< OccupancyTracker::STrack > &tracks )
#endif
{
	
	//
	// I'm going to try something _very_ different from 
	// my usual approach....
	//
	
	//
	// Question 1) How many objects do we think we have in the scene?
	//             My scenes tend to have a fairly constant number of people in them,
	//             so...
	std::vector<int> numDets( frameDetections.size() );
	int c = 0;
	for( auto fi = frameDetections.begin(); fi != frameDetections.end(); ++fi )
	{
		numDets[c] = fi->second.size();
		++c;
	}
	
	//
	// We could take the median and maybe miss a short lived object, or go more towards 
	// the top end or bottom end to get more or fewer tracks. So, that's probably 
	// our first tuning parameter.
	//
	std::sort( numDets.begin(), numDets.end() );
	float numTracksGuild = 0.6;
	int numTracks = numDets[ numTracksGuide * numDets.size()-1 ];
	
	cout << "Guessing that there's " << numTracks << " things worth trying to track. " << endl;
	
	
	
	//
	// Now there can be some kind of global optimisation - minimise the distance of tracks to 
	// detections, while minimising the mean distance a track moves.
	// That's probably intractable... but if we find a frame which looks like it 
	// reliably has our n detections for our n tracks, we can do 1-to-1 initialisation,
	// and then "flow" the tracks from frame to frame thereafter.
	//
	// but, where to start?
	//
	int bestRun      = 0;
	int bestRunStart = 0;
	int runStart = 0;
	bool inRun   = false;
	int startFrame = frameDetections.begin()->first;
	int lastFrame = (frameDetections.end()--)->first;
	for( auto fi = frameDetections.begin(); fi != frameDetections.end(); ++fi )
	{
		if( !inRun )
		{
			if( fi->second.size() == numTracks )
			{
				inRun = true;
				runStart = fi->first;
			}
		}
		else
		{
			if( fi->second.size() != numTracks || fi->first == lastFrame)
			{
				int runLen = fi->first - runStart;
				if( runLen > bestRun )
				{
					bestRun = runLen;
					bestRunStart = runStart;
				}
				inRun = false;
			}
		}
	}
	
	//
	// So, we initialise in the middle of the longest run.
	//
	int initFrame = bestRunStart + bestRun/2;
	auto fi = frameDetections.find( initFrame );
	while( fi == frameDetections.end() && initFrame > bestRunStart)
	{
		--initFrame;
		fi = frameDetections.find( initFrame );
	}
	if( fi == frameDetections.end() )
		throw std::runtime_error("Couldn't find init frame?" );
	
	int fc = fi->first;
	auto &dets = fi->second;
	assert( dets.size() == numTracks );
	
	tracks.resize( numTracks );
	for( unsigned dc = 0; dc < dets.size(); ++dc )
	{
		tracks[dc].framePeaks[ fc ] = dets[dc];
		tracks[dc].startFrame = fc;
		tracks[dc].endFrame = fc;
		tracks[dc].merged = -1;
	}
	
	//
	// Then we track forwards to the end...
	//
	for( int fc = initFrame+1; fc <= lastFrame; ++fc )
	{
		fi = frameDetections.find( fc );
		if( fi != frameDetections.end() )
			UpdateTracks( fc, tracks, dets );
	}
	
	
	//
	// And backwards to the start.
	//
	for( int fc = initFrame-1; fc >= 0; --fc )
	{
		fi = frameDetections.find( fc );
		if( fi != frameDetections.end() )
			UpdateTracks( fc, tracks, dets );
	}
	
	
	
#ifdef OCCTRACK_DEBUG
	bool paused, advance;
	paused = advance = false;
	for( auto fdi = frameDetections.begin(); fdi != frameDetections.end(); ++fdi )
	{
		dbgImg = cv::Mat( dbgImg.rows, dbgImg.cols, CV_32FC3, cv::Scalar(0,0,0) );
		
		for( unsigned dc = 0; dc < fdi->second.size(); ++dc )
		{
			hVec2D p = fdi->second[dc].mean();
			cv::circle( dbgImg, cv::Point( p(0), p(1) ), 4, cv::Scalar(0,0,0.5) );
		}
		
		for( unsigned tc = 0; tc < tracks.size(); ++tc )
		{
			auto fi = tracks[tc].framePeaks.find( fdi->first );
			hVec2D p = fi->second.mean();
			cv::circle( dbgImg, cv::Point( p(0), p(1) ), 4, cv::Scalar(0,1.0,0) );
		}
		
		
		
		dbgRen->Step( paused, advance );
		while( paused && !advance )
			dbgRen->Step( paused, advance );
	}
		
#endif
}




void OccupancyTracker::FitGaussian( cv::Mat img, int x0, int y0, int x1, int y1, hVec2D &mean, Eigen::Matrix2f &cov, float &maxVal )
{
	//
	// We can easily compute the mean of the points,
	// and indeed, use the occupancy values to weight that mean.
	//
	mean << 0,0,0;
	maxVal = 0.0f;
	for( unsigned rc = y0; rc < y1; ++rc )
	{
		for( unsigned cc = x0; cc < x1; ++cc )
		{
			float &i = img.at<float>(rc,cc);
			hVec2D p;
			p << i*cc,i*rc,i;
			mean += p;
			maxVal = std::max( maxVal, i );
		}
	}
	float weightSum = mean(2);
	mean /= weightSum;
	
	//
	// Now I want to compute a weighted 2D variance. Actually,
	// what I really want is the covariance, not just a variance -
	// I want a vague representation of the shape of the blob.
	//
	// Basic weighted variance is sum( w * || p - m ||^2 )
	// assuming the weights sum to 1. But I want covariance...
	// oh... I see...
	//
	cov << 0,0,0,0;
	for( unsigned rc = y0; rc < y1; ++rc )
	{
		for( unsigned cc = x0; cc < x1; ++cc )
		{
			float &i = img.at<float>(rc,cc);
			float w = i / weightSum;
			
			float xd = cc - mean(0);
			float yd = rc - mean(1);
			cov(0,0) += w * (xd*xd);
			cov(0,1) += w * (xd*yd);
			cov(1,0) += w * (yd*xd);
			cov(1,1) += w * (yd*yd);
		}
	}
	
	// right? right?
}
