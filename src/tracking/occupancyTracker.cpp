#include "occupancyTracker.h"

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

float SpatialDist( OccupancyTracker::SPeak &a, OccupancyTracker::SPeak &b )
{
	// we should take into account the covariances I'm sure, but that needs 
	// more reading - it is not as simple as a Mahalanobis distance.
	return (a.mean-b.mean).norm();
}

void TrackDistanceSFM( OccupancyTracker::STrack &singleFrameTrack, OccupancyTracker::STrack &multiFrameTrack, float &sd, float &td )
{
	
	//
	// The simple thing is to find the closest point in time between b and a
	//
	
	// singleFrameTrack happens before multiFrameTrack starts
	if( singleFrameTrack.startFrame <= multiFrameTrack.startFrame )
	{
		sd = SpatialDist( multiFrameTrack.framePeaks[ multiFrameTrack.startFrame ], singleFrameTrack.framePeaks[ singleFrameTrack.startFrame ] );
		td = abs( multiFrameTrack.startFrame - singleFrameTrack.startFrame );
	}
	
	// singleFrameTrack happens after multiFrameTrack ends
	else if( singleFrameTrack.startFrame >= multiFrameTrack.endFrame )
	{
		sd = SpatialDist( multiFrameTrack.framePeaks[ multiFrameTrack.endFrame ], singleFrameTrack.framePeaks[ singleFrameTrack.startFrame ] );
		td = abs( multiFrameTrack.endFrame - singleFrameTrack.startFrame );
	}
	
	// singleFrameTrack happens during multiFrameTrack
	else
	{
		// we'll just quickly search for the nearest moment.
		int bestFrame = multiFrameTrack.startFrame;
		float bestTD  = abs( multiFrameTrack.startFrame - singleFrameTrack.startFrame );
		for( auto afi = multiFrameTrack.framePeaks.begin(); afi != multiFrameTrack.framePeaks.end(); ++afi )
		{
			float tmpTD = abs( afi->first - singleFrameTrack.startFrame );
			if( tmpTD < bestTD )
			{
				bestFrame = afi->first;
				bestTD = tmpTD;
			}
		}
		
		sd = SpatialDist( multiFrameTrack.framePeaks[ bestFrame ], singleFrameTrack.framePeaks[ singleFrameTrack.startFrame ] );
		td = bestTD;
	}
	
}

void TrackDistanceCorrelateDuringOverlap( OccupancyTracker::STrack &a, OccupancyTracker::STrack &b, float &sd, float &td )
{
	//
	// Tracking literature is just far too large for my liking - so there are an infinite number of 
	// things I'm ignoring as I naively plow on with my own tracking approach. That's just... the way 
	// it has to be. 
	//
	
	
	// One option is the mean distance between the tracks during the period of their overlap, we should probably
	// be factoring in the covariance of each peak and indeed the variance of the difference...
	float meanD = 0.0f;
	int cnt = 0;
	unsigned sf = std::max( a.startFrame, b.startFrame );
	unsigned ef = std::min( a.endFrame, b.endFrame );
	for( unsigned fc = sf; fc <= ef; ++fc )
	{
		auto pa = a.framePeaks.find( fc );
		auto pb = b.framePeaks.find( fc );
		if( pa != a.framePeaks.end() && pb != b.framePeaks.end() )
		{
			meanD += ( pa->second.mean - pb->second.mean ).norm();
			++cnt;
		}
	}
	meanD /= cnt;
	
	float var = 0.0f;
	for( unsigned fc = sf; fc <= ef; ++fc )
	{
		auto pa = a.framePeaks.find( fc );
		auto pb = b.framePeaks.find( fc );
		if( pa != a.framePeaks.end() && pb != b.framePeaks.end() )
		{
			float d = ( pa->second.mean - pb->second.mean ).norm();
			var += (d-meanD)*(d-meanD);
		}
	}
	var /= cnt;
	float std = sqrt(var);
	
	
	// experiment:
	sd = meanD + var;
	td = 0;
	
}

float TrackDistance( OccupancyTracker::STrack &a, OccupancyTracker::STrack &b, float &sd, float &td )
{
	if( a.merged >= 0 || b.merged >= 0 )
		return 99999.7f;
	//
	// What is a rational distance between tracks? 
	//
	// This is a complicated thing to consider, because we have lots of scenarios.
	//
	//   - Consider if we're looking at single frames, track vs. single frame, track vs. track
	//   - do the two tracks overlap in time?
	// 
	sd = td = 999999.99f;
	
	int ostart = std::max( a.startFrame, b.startFrame );
	int oend   = std::min( a.endFrame, b.endFrame );
	
	
	//
	// Both tracks are single frames. 
	//
	if( a.endFrame - a.startFrame == 0 && b.endFrame - b.startFrame == 0)
	{
		sd = SpatialDist( a.framePeaks[ a.startFrame ], b.framePeaks[ b.startFrame ] );
		td = abs( a.startFrame - b.startFrame );
	}
	
	//
	// a is multiple frames, b is a single frame.
	//
	else if( a.endFrame - a.startFrame > 0 && b.endFrame - b.startFrame == 0)
	{
		TrackDistanceSFM( b, a, sd, td );
	}
	
	//
	// a is single frame, b is multiple frames
	//
	else if( a.endFrame - a.startFrame == 0 && b.endFrame - b.startFrame > 0)
	{
		TrackDistanceSFM( a, b, sd, td );
	}
	
	
	//
	// Both tracks cover multiple frames.
	//
	else if( a.endFrame - a.startFrame > 0 && b.endFrame - b.startFrame > 0)
	{
		// if they overlap in time, take the mean distance over that overlap
		if( a.startFrame <= b.endFrame && a.endFrame >= b.startFrame )
		{
			TrackDistanceCorrelateDuringOverlap( a, b, sd, td );
		}
		
		// if they don't overlap in time, take the distance at closest time
		else if( a.endFrame < b.startFrame )
		{
			sd = SpatialDist( a.framePeaks[ a.endFrame ], b.framePeaks[ b.startFrame ] );
			td = b.startFrame - a.endFrame;
		}
		else if( b.endFrame < a.startFrame )
		{
			sd = SpatialDist( a.framePeaks[ a.startFrame ], b.framePeaks[ b.endFrame ] );
			td = a.startFrame - b.endFrame;
		}
		
	}
	else
	{
		throw( std::runtime_error("How can this happen?") );
	}
	
	//
	// What is a distance that meaningfully handles spatial difference and time difference?
	// Being a long way apart in time should make merging quickly impossible, but being 
	// within a few frames, shouldn't be a major penalty.
	//
	
	//return  sd + exp(td-4);
	return sd * (1+exp(td-4));
}



void MergeTracks( int idxa, OccupancyTracker::STrack &tracka, OccupancyTracker::STrack &trackb )
{
	
	
	//
	// The first thing we need to know are the frames over which the tracks exist,
	// this is the union of the frames, not the intersection - we want to know all the frames.
	//
	std::set<int> frames;
	for( auto fi = tracka.framePeaks.begin(); fi != tracka.framePeaks.end(); ++fi )
	{
		frames.insert( fi->first );
	}
	for( auto fi = trackb.framePeaks.begin(); fi != trackb.framePeaks.end(); ++fi )
	{
		frames.insert( fi->first );
	}
	
	//
	// Now we need to update the peaks in track a with information from 
	// track b.
	// TODO: Be a lot more clever than this.
	//
	for( auto fi = frames.begin(); fi != frames.end(); ++fi )
	{
		auto fi0 = tracka.framePeaks.find( *fi );
		auto fi1 = trackb.framePeaks.find( *fi );
		
		if( fi0 != tracka.framePeaks.end() && fi1 != trackb.framePeaks.end() )
		{
			// both tracks have this frame.
			// get a new mean and covariance.
			hVec2D newMean = ( (fi0->second.mean * fi0->second.area) + (fi1->second.mean * fi1->second.area) )
			                 / (fi0->second.area + fi1->second.area );
			
			Eigen::Matrix2f newCov = ( (fi0->second.cov * fi0->second.area) + (fi1->second.cov * fi1->second.area) )
			                 / (fi0->second.area + fi1->second.area );
			
			fi0->second.mean = newMean;
			fi0->second.cov  = newCov;
			
		}
		else if( fi0 == tracka.framePeaks.end() && fi1 != trackb.framePeaks.end() )
		{
			// only b has this frame
			tracka.framePeaks[ *fi ] = trackb.framePeaks[ *fi ];
		}
		else if( fi0 != tracka.framePeaks.end() && fi1 != trackb.framePeaks.end() )
		{
			// only a has this frame.
			// do nothing.
		}
	}
	
	//
	// say that b got merged into a.
	//
	trackb.merged = idxa;
	
	//
	// update the start and end frame of a
	//
	tracka.startFrame = std::min( tracka.startFrame, trackb.startFrame );
	tracka.endFrame   = std::max(   tracka.endFrame,   trackb.endFrame );
	
	//
	// Fill gaps in the track.
	// Its probably just one or two frames here and there. Mostly...
	//
	// Why do it? Because it helps when getting the distance between tracks, and it 
	// also helps when merging tracks together. Probably.
	//
	//
	std::vector< std::pair<int,int> > gaps;
	auto fi = tracka.framePeaks.begin();
	int i = fi->first;
	while( fi != tracka.framePeaks.end() )
	{
		// my memory says that the keys in a map will be sorted so this should work...
		if( fi->first - i > 1 )
		{
			gaps.push_back( std::pair<int,int>( i, fi->first ) );
		}
		i = fi->first;
		++fi;
	}
	
	cout << "gaps: " << endl;
	for( unsigned gc = 0; gc < gaps.size(); ++gc )
	{
		//
		// Just a simple linear interpolation.
		//
		int a = gaps[gc].first;
		int b = gaps[gc].second;
		
		cout << "\t" << a << " -> " << b << endl;
		
		OccupancyTracker::SPeak &pa = tracka.framePeaks[ a ];
		OccupancyTracker::SPeak &pb = tracka.framePeaks[ b ];
		hVec2D d = pb.mean - pa.mean;
		Eigen::Matrix2f covd = pb.cov - pa.cov;
		
		cout << "\t" << pa.mean.transpose() << endl;
		cout << "\t" << pb.mean.transpose() << endl;
		cout << pa.cov << endl;
		cout << pb.cov << endl;
		
		int t = b - a;
		int ad = pb.area - pa.area;
		
		for( int c = a+1; c < b; ++c )
		{
			float k = (float)(c-a)/(float)t;
			OccupancyTracker::SPeak pc;
			pc.mean = pa.mean + d * k;
			pc.mean(2) = 1.0f;
			pc.cov = pa.cov + covd * k;
			pc.area = pa.area + ad * k;
			
			cout << "\t\t" << c << endl;
			cout << "\t\t" << pc.mean.transpose() << endl;
			cout << pc.cov.transpose() << endl;
			
			tracka.framePeaks[ c ] = pc;
		}
	}
	
	
}

void MergeTracks( std::pair<int,int> p, std::vector< OccupancyTracker::STrack > &tracks )
{
	int a = p.first;
	int b = p.second;
	
	cout << "merging: " << a << " <- " << b << endl;
	cout << tracks[a].startFrame << " " << tracks[a].endFrame << " : " << tracks[b].startFrame << " " << tracks[b].endFrame << endl;
	
	MergeTracks( a, tracks[a], tracks[b] );
}

void OccupancyTracker::SimpleAssociate( std::vector< OccupancyTracker::STrack > &tracks, int frame )
{
	// subset of tracks that exist in most recent previous frame
	std::vector<int> atracks;
	for( unsigned tc = 0; tc < tracks.size(); ++tc )
	{
		if( tracks[tc].endFrame == frame - 1 )
			atracks.push_back(tc);
	}
// 	cout << "----" << endl;
	// create "track" for peaks in current frame.
	std::vector< STrack > ntracks;
	auto fi0 = frameDetections.find( frame );
	for( unsigned pc = 0; pc < fi0->second.size(); ++pc )
	{
		STrack t;
		t.framePeaks[ fi0->first ] = fi0->second[pc];
		t.startFrame = fi0->first;
		t.endFrame   = fi0->first;
		t.merged     = -1;
		
		ntracks.push_back(t);
		
// 		cout << pc << " " << t.framePeaks[fi0->first].mean.transpose() << endl;
	}
	
	float sd,td;
	if( tracks.size() == 0 )
	{
		tracks = ntracks;
	}
	else
	{
		genMatrix D( atracks.size(), ntracks.size() );
		
		for( unsigned atc = 0; atc < atracks.size(); ++atc )
		{
			unsigned tc = atracks[ atc ];
			for( unsigned ntc = 0; ntc < ntracks.size(); ++ntc )
			{
				D( atc, ntc ) = TrackDistance( tracks[tc], ntracks[ntc], sd, td );
			}
		}
		
		std::vector< int > assocs( ntracks.size(), -1 );
		float bestDist;
		do
		{
			int atc, ntc;
			bestDist = D.minCoeff( &atc, &ntc );
			
			// is this an unambiguous association?
			// i.e. what is the second best track association for this detection?
			//      what is the second best detection association for this track?
			float nextBestTrack = 9999999.99f, nextBestDet = 999999.99f;
			for( unsigned atc2 = 0; atc2 < atracks.size(); ++atc2 )
			{
				if( atc2 != atc && D(atc2,ntc) < nextBestTrack )
					nextBestTrack = D( atc2, ntc );
			}
			for( unsigned ntc2 = 0; ntc2 < ntracks.size(); ++ntc2 )
			{
				if( ntc2 != ntc && D(atc,ntc2) < nextBestDet )
					nextBestDet = D( atc, ntc2 );
			}
			
			if( bestDist < settings.strictDistanceThreshold && bestDist < 0.25 * nextBestDet && bestDist < 0.25 * nextBestTrack )
			{
				assocs[ ntc ] = atc;
			}
			
			// no unambiguous association for this det
			for( unsigned atc2 = 0; atc2 < atracks.size(); ++atc2 )
				D( atc2, ntc ) = 9999999.99f;
			for( unsigned ntc2 = 0; ntc2 < ntracks.size(); ++ntc2 )
				D( atc, ntc2 ) = 9999999.99f;
		}
		while( bestDist < settings.strictDistanceThreshold );
		
		for( unsigned ntc = 0; ntc < assocs.size(); ++ntc )
		{
			if( assocs[ntc] >= 0 )
			{
				int tc = atracks[ assocs[ntc] ];
				MergeTracks( tc, tracks[tc], ntracks[ntc] );
			}
			else
			{
				tracks.push_back( ntracks[ntc] );
			}
		}
	}
	
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


#ifdef OCCTRACK_DEBUG
void OccupancyTracker::GetTracks( std::vector< OccupancyTracker::STrack > &tracks, std::shared_ptr<Rendering::BasicPauseRenderer> &dbgRen, cv::Mat &dbgImg )
#else
void OccupancyTracker::GetTracks( std::vector< OccupancyTracker::STrack > &tracks )
#endif
{
	//
	// Given all the detections over all the frames, we could just turn each detection into a 1-frame tracklet,
	// and then do some kind of clustering to merge those tracklets together. 
	//
	// Trouble with that is that it quickly becomes absurd - but can be made a lot easier because from 
	// frame to frame there are a lot of unambiguous temporal associations.
	//
	
	//
	// First off, turn the detections on the first frame into 1-frame tracks.
	//
	cout << "Simple associate: " << endl;
	std::vector< STrack > ptracks;
	auto fi0 = frameDetections.begin();
	for( auto fi0 = frameDetections.begin(); fi0 != frameDetections.end(); ++fi0 )
	{
		SimpleAssociate( ptracks, fi0->first );
		cout << "frame " << fi0->first << " : " << "total tracks: " << ptracks.size() << endl;
	}
	
	
	//
	// Now we want some sort of iterative process that merges the ptracks together until
	// it becomes too "expensive" to merge them together.
	//
	// Quality tracking has a myriad awkward things to solve, like splits and merges 
	// and crossings... just how sophisticated do we want to get here, when the truth
	// of that matter is that I pretty much only ever have one person in the scene?
	//
	
	
	//
	// We're basically trying to make a minimum spanning tree through all the initial points,
	// making one single track - with the caveat being that at some point we'll say tracks 
	// are too far apart to be merged together.
	//
	// The thing that makes this more difficult than just a basic MST is that the distance between
	// tracks is more complicated than simply the distance between their two closest elements. Well,
	// in my opinion there is anyway. That means that once you merge two tracks together, their 
	// distance to other tracks ends up changing. 
	//
	//
	std::map< std::pair<int,int>, float > distances;
	std::vector< std::vector<std::pair<int,int> > > track2distanceTable;
	std::pair<int,int> closestPair(0,1);
#ifdef OCCTRACK_DEBUG
	std::ofstream tmpfi("dists");
#endif
	float sd,td;
	
	track2distanceTable.resize( ptracks.size() );
	
	for( unsigned tc0 = 0; tc0 < ptracks.size(); ++tc0 )
	{
		for( unsigned tc1 = tc0+1; tc1 < tc0 + settings.numNearPeaks && tc1 < ptracks.size(); ++tc1 )
		{
			// we'll limit the number of other tracks that we can link to,
			// we could do this by limiting the difference in time, but here
			// I'm just going to limit the difference between t0 and t1, knowing
			// that tc1 and tc0 are in time order anyway. This'll only fail if we 
			// have too many detections in one frame and fail to span time.
			
			auto p = std::pair<int,int>(tc0,tc1);
			float d = TrackDistance( ptracks[tc0], ptracks[tc1], sd, td );
			
			distances[p] = d;
			if( distances[p] < distances[ closestPair ] )
			{
				closestPair = p;
			}
			track2distanceTable[ tc0 ].push_back( p );
			track2distanceTable[ tc1 ].push_back( p );
			
#ifdef OCCTRACK_DEBUG
			tmpfi << tc0 << " " << tc1 << " : " << distances[p] << " : " << sd << " " << td << endl;
#endif
			
		}
	}
#ifdef OCCTRACK_DEBUG
	tmpfi.close();
#endif
	
	
	
	
	
	
	bool paused, advance;
	paused = advance = false;
	int fc0 = 0;
	int fc1 = 6000;
	int maxTimeGap = 50;
	for( unsigned fc = fc0; fc < fc1; ++fc )
	{
		std::vector<int> activeTracks;
		for( unsigned tc = 0; tc < ptracks.size(); ++tc )
		{
			if( fc >= ptracks[tc].startFrame-maxTimeGap && fc <= ptracks[tc].endFrame+maxTimeGap )
				activeTracks.push_back( tc );
		}
		
		genMatrix D( activeTracks.size(), activeTracks.size() );
		for( unsigned atc0 = 0; atc0 < D.rows(); ++atc0 )
		{
			for( unsigned atc1 = 0; atc1 < D.cols(); ++atc1 )
			{
				if( atc1 <= atc0 )
					D(atc0, atc1) = 99999999.99f;
				else
					D(atc0, atc1) = TrackDistance( ptracks[ activeTracks[atc0] ], ptracks[activeTracks[atc1] ], sd, td );
			}
		}
		
		cout << D.rows() << endl;
		
		//
		//
		//
		
		
		
#ifdef OCCTRACK_DEBUG
		dbgImg = cv::Mat( dbgImg.rows, dbgImg.cols, CV_32FC3, cv::Scalar(0,0,0) );
		for( unsigned atc = 0; atc < activeTracks.size(); ++atc )
		{
			int tc = activeTracks[atc];
			// draw a line through all peaks in this track - but what colour?
			float b,g,r;
			if( fc < ptracks[tc].startFrame )
			{
				g = 0;
				r = 0;
				b = std::max( 0.25, 1.0 - (ptracks[tc].startFrame-fc)/100.0f );
			}
			else if( fc >= ptracks[tc].startFrame && fc <= ptracks[tc].endFrame )
			{
				g = 1.0f;
				b = 0.0f;
				r = 0.0f;
			}
			else if( fc > ptracks[tc].endFrame )
			{
				g = 0.0f;
				b = 0.0f;
				r = std::max( 0.25, 1.0 - (fc-ptracks[tc].endFrame)/100.0f );
			}
			
			auto fi0 = ptracks[tc].framePeaks.begin();
			auto fi1 = ptracks[tc].framePeaks.begin();
			auto fi  = ptracks[tc].framePeaks.find( fc );
			fi1++;
			
			if( fi != ptracks[tc].framePeaks.end() )
			{
				cv::Point p0( fi->second.mean(0), fi->second.mean(1) );
				cv::circle( dbgImg, p0, 2, cv::Scalar(0,1,0) );
			}
			
			while( fi1 != ptracks[tc].framePeaks.end() )
			{
				cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
				cv::Point p1( fi1->second.mean(0), fi1->second.mean(1) );
				
				cv::line( dbgImg, p0, p1, cv::Scalar(b,g,r), 2 );
				++fi0;
				++fi1;
			}
		}
		
		
		dbgRen->SetBGImage( dbgImg );
		dbgRen->Step(paused, advance);
		while( paused && !advance )
			dbgRen->Step(paused, advance);
		advance = false;
#endif
	}
	
	

	
	
	
	
	
	paused = advance = false;
	
#ifdef OCCTRACK_DEBUG
	std::ofstream mfi("merges");
#endif
	bool done = false;
	while( !done )
	{
		
		//
		// You can tell I'm grasping at straws
		//
		
		
		
		
		
		
		std::pair<int,int> merge = closestPair;
		
#ifdef OCCTRACK_DEBUG
		dbgImg = cv::Mat( dbgImg.rows, dbgImg.cols, CV_32FC3, cv::Scalar(0,0,0) );
		cout << "tracks: " << ptracks.size() << endl;
		float sd,td;
		float d = TrackDistance( ptracks[ merge.first ], ptracks[ merge.second ], sd, td );
		cout << "merge: " << merge.first << " " << merge.second << " : " << distances[ merge ] << " ( " << d << " " << sd << " " << td  << " ) " << endl;
		
		mfi << "---------" << endl;
		mfi << "merge: " << merge.first << " " << merge.second << " : " << distances[ merge ] << endl;
		mfi << "\tsizes: " << ptracks[ merge.first ].framePeaks.size() << " " << ptracks[ merge.second ].framePeaks.size() << endl;
#endif
		
		MergeTracks( merge, ptracks );
		
		
		
		//
		// Update the distances and find out what the new closest pair is.
		//
		// TODO: We don't want to search through all the distances looking for the ones that need updating.
		//       instead, we should have a table saying which distances are relevant for each track. 
		//
		
		
		// so, the second track in the merge pair no longer exists, but it could have a
		// distance to another track that the first track didn't have.
		std::set< std::pair<int,int> > toUpdate;
// 		cout << " A : ";
		for( unsigned pc = 0; pc < track2distanceTable[ merge.first ].size(); ++pc )
		{
			toUpdate.insert( track2distanceTable[ merge.first ][pc] );
// 			cout << "( " << track2distanceTable[ merge.first ][pc].first << " " << track2distanceTable[ merge.first ][pc].second << " ) ";
		}
// 		cout << endl;
// 		cout << "B : "; 
		for( unsigned pc = 0; pc < track2distanceTable[ merge.second ].size(); ++pc )
		{
			auto p = track2distanceTable[ merge.second ][pc];
			
// 			cout << "( " << p.first << " " << p.second << " ) ";
			
			std::pair<int,int> np;
			if( p.first == merge.second )
			{
				np.first  = std::min( p.second, merge.first );
				np.second = std::max( p.second, merge.first );
			}
			else if( p.second = merge.second )
			{
				np.first  = std::min( p.first, merge.first );
				np.second = std::max( p.first, merge.first );
			}
			
// 			cout << " | " << p.first << " " << " | " << p.second << endl;
			
			toUpdate.insert( p );
		}
		track2distanceTable[ merge.first ].clear();
		for( auto i = toUpdate.begin(); i != toUpdate.end(); ++i )
			track2distanceTable[ merge.first ].push_back(*i);
		
		
		// now we can update those few distances.
		for( unsigned pc = 0; pc < track2distanceTable[ merge.first ].size(); ++pc )
		{
			auto p = track2distanceTable[ merge.first ][pc];
			distances[ p ] = TrackDistance( ptracks[ p.first ], ptracks[ p.second ], sd, td );
#ifdef OCCTRACK_DEBUG
			// mfi << "  new dist: " << p.first << ", " << p.second << " -> " << distances[ p ] << "(" << sd << ", " << td << ") " << endl;
#endif
		}
		
		distances[ merge ] = 999999.99;
		
#ifdef OCCTRACK_DEBUG
		cout << "update" << endl;
#endif
		float closestDist = 999999.99;
		for( auto di = distances.begin(); di != distances.end(); ++di )
		{
			if( di->second < closestDist )
			{
				closestDist = di->second;
				closestPair = di->first;
			}
		}
		
#ifdef OCCTRACK_DEBUG
		mfi << "next merge: " << closestPair.first << " " << closestPair.second << " : " << distances[ closestPair ] << endl;
		mfi << "\t" << ptracks[ closestPair.first ].startFrame << " " << ptracks[ closestPair.first ].endFrame << endl;
		mfi << "\t" << ptracks[ closestPair.second ].startFrame << " " << ptracks[ closestPair.second ].endFrame << endl;
#endif
		
		
		
#ifdef OCCTRACK_DEBUG
		//
		// Debug render of the existing tracks.
		//
		cout << "dbg ren" << endl;
		for( unsigned tc = 0; tc < ptracks.size(); ++tc )
		{
			if( ptracks[tc].merged >= 0 )
				continue;
			auto fi0 = ptracks[tc].framePeaks.begin();
			auto fi1 = ptracks[tc].framePeaks.begin();
			fi1++;
			
			cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
			cv::circle( dbgImg, p0, 2, cv::Scalar(0,1,0) );
			
			
			int npeaks = std::min( (size_t)200, ptracks[tc].framePeaks.size() );
			//float b = npeaks / 200.0f;
			float b = tc / (float)ptracks.size();
			float r = 1.0 - b;
			while( fi1 != ptracks[tc].framePeaks.end() )
			{
				cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
				cv::Point p1( fi1->second.mean(0), fi1->second.mean(1) );
				
				
				
				cv::line( dbgImg, p0, p1, cv::Scalar(b,0,r), 2 );
				++fi0;
				++fi1;
			}
		}
		
		dbgRen->SetBGImage( dbgImg );
		dbgRen->Step(paused, advance);
		while( paused && !advance )
			dbgRen->Step(paused, advance);
		advance = false;
#endif
		cout << closestDist << " >? " << settings.distanceThreshold << " " << (closestDist > settings.distanceThreshold) << endl;
		done = closestDist > settings.distanceThreshold;
	}

#ifdef OCCTRACK_DEBUG
	paused = advance = false;
	while( 1 )
	{
		cout << "dbg ren" << endl;
		for( unsigned tc = 0; tc < ptracks.size(); ++tc )
		{
			if( ptracks[tc].merged >= 0 )
				continue;
			
			auto fi00 = ptracks[tc].framePeaks.begin();
			while( fi00 != ptracks[tc].framePeaks.end() )
			{
				dbgImg = cv::Mat( dbgImg.rows, dbgImg.cols, CV_32FC3, cv::Scalar(0,0,0) );
				
				auto fi0 = ptracks[tc].framePeaks.begin();
				auto fi1 = ptracks[tc].framePeaks.begin();
				fi1++;
				
				cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
				cv::circle( dbgImg, p0, 2, cv::Scalar(0,1,0) );
				
				
				int npeaks = std::min( (size_t)200, ptracks[tc].framePeaks.size() );
				float b = npeaks / 200.0f;
				float r = 1.0 - b;
				while( fi1 != ptracks[tc].framePeaks.end() )
				{
					cv::Point p0( fi0->second.mean(0), fi0->second.mean(1) );
					cv::Point p1( fi1->second.mean(0), fi1->second.mean(1) );
					
					
					
					cv::line( dbgImg, p0, p1, cv::Scalar(b,0,r), 2 );
					++fi0;
					++fi1;
				}
				
				DrawEllipse( dbgImg, fi00->second, cv::Scalar(0,1,1), 2 );
				dbgRen->SetBGImage( dbgImg );
				dbgRen->Step(paused, advance);
				while( paused && !advance )
					dbgRen->Step(paused, advance);
				advance = false;
				fi00++;
				
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}
	}
#endif
	
	for( unsigned tc = 0; tc < ptracks.size(); ++tc )
	{
		if( ptracks[tc].merged < 0 )
			tracks.push_back( ptracks[tc] );
	}
	
	return;
	
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
