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
	//   - do the two tracks overlap in time
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
	// b is either before, during, or after a.
	// Find the spatial distance at the most appropriate time.
	//
	else if( a.endFrame - a.startFrame > 0 && b.endFrame - b.startFrame == 0)
	{
		
		if( b.startFrame <= a.startFrame )
		{
			sd = SpatialDist( a.framePeaks[ a.startFrame ], b.framePeaks[ b.startFrame ] );
			td = abs( a.startFrame - b.startFrame );
		}
		else if( b.startFrame >= a.endFrame )
		{
			sd = SpatialDist( a.framePeaks[ a.endFrame ], b.framePeaks[ b.startFrame ] );
			td = abs( a.endFrame - b.startFrame );
		}
		else
		{
			// TODO: I don't really want to be searching for the nearest time.
			//       can a track have a gap in time - or does the merge interpolate that gap?
			//       right now, there's a gap :(
			int bestFrame = a.startFrame;
			float bestTD  = abs( a.startFrame - b.startFrame );
			for( auto afi = a.framePeaks.begin(); afi != a.framePeaks.end(); ++afi )
			{
				float tmpTD = abs( afi->first - b.startFrame );
				if( tmpTD < bestTD )
				{
					bestFrame = afi->first;
					bestTD = tmpTD;
				}
			}
			
			sd = SpatialDist( a.framePeaks[ bestFrame ], b.framePeaks[ b.startFrame ] );
			td = bestTD;
		}
	}
	
	
	//
	// a is single frame, b is multiple frames
	//
	// a is either before, during, or after b.
	// Find the spatial distance at the most appropriate time.
	// NOTE: Code duplication of above: make a function!
	//
	else if( a.endFrame - a.startFrame == 0 && b.endFrame - b.startFrame > 0)
	{
		if( a.startFrame <= b.startFrame )
		{
			sd = SpatialDist( b.framePeaks[ b.startFrame ], a.framePeaks[ a.startFrame ] );
			td = abs( b.startFrame - a.startFrame );
		}
		else if( a.startFrame >= b.endFrame )
		{
			sd = SpatialDist( b.framePeaks[ b.endFrame ], a.framePeaks[ a.startFrame ] );
			td = abs( b.endFrame - a.startFrame );
		}
		else
		{
			// TODO: I don't really want to be searching for the nearest time.
			//       can a track have a gap in time - or does the merge interpolate that gap?
			//       right now, there's a gap :(
			int bestFrame = b.startFrame;
			float bestTD  = abs( b.startFrame - a.startFrame );
			for( auto bfi = b.framePeaks.begin(); bfi != b.framePeaks.end(); ++bfi )
			{
				float tmpTD = abs( bfi->first - a.startFrame );
				if( tmpTD < bestTD )
				{
					bestFrame = bfi->first;
					bestTD = tmpTD;
				}
			}
			
			sd = SpatialDist( b.framePeaks[ bestFrame ], a.framePeaks[ a.startFrame ] );
			td = bestTD;
		}
	}
	
	
	//
	// Both tracks cover multiple frames.
	//
	// We'll set the distance between them as the mean distance 
	// during the overlap, if there is one. Otherwise the difference
	// and nearest time point.
	//
	// TODO: We could also consider the direction of the track?
	//
	else if( a.endFrame - a.startFrame > 0 && b.endFrame - b.startFrame > 0)
	{
		// if they overlap in time, take the mean distance over that overlap
		if( a.startFrame <= b.endFrame && a.endFrame >= b.startFrame )
		{
			// TODO: Again we have the problem of there maybe being a gap in the track.
			//       but it gets worse here because there's the horrible chance,
			//       however small, of there not being any actual shared frames. Oups.
			int cnt = 0;
			float mean = 0.0f;
			int closest = 99999;
			for( int f = std::max( a.startFrame, b.startFrame ); f <= std::min(a.endFrame, b.endFrame); ++f )
			{
				auto afi = a.framePeaks.find( f );
				auto bfi = b.framePeaks.find( f );
				if( afi != a.framePeaks.end() && bfi != b.framePeaks.end() )
				{
					cnt++;
					mean += SpatialDist( afi->second, bfi->second );
				}
				
			}
			
			if( cnt == 0 )
			{
				// awkwardly, they overlap in time, but don't have detections on the
				// same frame...
				//
				// so we can linearly interpolate each tracklet to fill the gaps...
				// then compute the distances again.
				// So that's our big TODO.
				// well we've fused elsewhere...
				
				
				cout << "cnt == 0" << endl;
				cout << a.startFrame << " " << a.endFrame << " : ";
				for( auto afi = a.framePeaks.begin(); afi != a.framePeaks.end(); ++afi )
					cout << afi->first << " " ;
				cout << endl;
				
				cout << b.startFrame << " " << b.endFrame << " : ";
				for( auto bfi = b.framePeaks.begin(); bfi != b.framePeaks.end(); ++bfi )
					cout << bfi->first << " " ;
				cout << endl;
				
				exit(0);
			}
			sd = mean / cnt;
			td = 0;
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
	// How does being 1 frame apart compare to being 1 cell apart?
	// It seems like we want time distance to be less important than space difference,
	// but I imagine that, after some point, you'd want it to be much larger. 
	// Kind of an awkward tradeoff.
	//
	return  sd + log( std::max(1.0f,td) );
}

void MergeTracks( std::pair<int,int> p, std::vector< OccupancyTracker::STrack > &tracks )
{
	int a = p.first;
	int b = p.second;
	
	cout << "merging: " << a << " <- " << b << endl;
	cout << tracks[a].startFrame << " " << tracks[a].endFrame << " : " << tracks[b].startFrame << " " << tracks[b].endFrame << endl;
	
	//
	// The first thing we need to know are the frames over which the tracks exist,
	// this is the union of the frames, not the intersection - we want to know all the frames.
	//
	std::set<int> frames;
	for( auto fi = tracks[a].framePeaks.begin(); fi != tracks[a].framePeaks.end(); ++fi )
	{
		frames.insert( fi->first );
	}
	for( auto fi = tracks[b].framePeaks.begin(); fi != tracks[b].framePeaks.end(); ++fi )
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
		auto fi0 = tracks[a].framePeaks.find( *fi );
		auto fi1 = tracks[b].framePeaks.find( *fi );
		
		if( fi0 != tracks[a].framePeaks.end() && fi1 != tracks[b].framePeaks.end() )
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
		else if( fi0 == tracks[a].framePeaks.end() && fi1 != tracks[b].framePeaks.end() )
		{
			// only b has this frame
			tracks[a].framePeaks[ *fi ] = tracks[b].framePeaks[ *fi ];
		}
		else if( fi0 != tracks[a].framePeaks.end() && fi1 != tracks[b].framePeaks.end() )
		{
			// only a has this frame.
			// do nothing.
		}
	}
	
	//
	// say that b got merged into a.
	//
	tracks[ p.second ].merged = p.first;
	
	//
	// update the start and end frame of a
	//
	tracks[ p.first ].startFrame = std::min( tracks[ p.first ].startFrame, tracks[p.second].startFrame );
	tracks[ p.first ].endFrame   = std::max(   tracks[ p.first ].endFrame,   tracks[p.second].endFrame );
	
	//
	// Fill gaps in the track.
	// Its probably just one or two frames here and there. Mostly...
	//
	// Why do it? Because it helps when getting the distance between tracks, and it 
	// also helps when merging tracks together. Probably.
	//
	//
	std::vector< std::pair<int,int> > gaps;
	auto fi = tracks[ p.first ].framePeaks.begin();
	int i = fi->first;
	while( fi != tracks[ p.first ].framePeaks.end() )
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
		
		OccupancyTracker::SPeak &pa = tracks[ p.first ].framePeaks[ a ];
		OccupancyTracker::SPeak &pb = tracks[ p.first ].framePeaks[ b ];
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
			
			tracks[ p.first ].framePeaks[ c ] = pc;
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
	// Tracking is inevitably a frame-to-frame type thing, but frame-to-frame associations 
	// is somewhat low-performing. Our task is to look at all the data that we have and 
	// try to get the best possible solution with as few noise responses as possible and 
	// I don't care how many times we pass over the data.
	//
	// What I forsee is basically a clustering problem.
	// Start with the extreme case - every individual detection in every frame 
	// is its own tracklette
	//
	// Compute the distances between those "tracklettes" where distance is a function
	// of space and time, taking into account the Gaussian we fit to the location.
	//
	// Iteratively merge together tracklettes. Obviously, we'll end up with a 
	// hierarchical clustering through time which could lead to one single track if
	// we don't know when to stop. So the trick is knowing when to stop,
	// or at least, at what level to cut the hierarchy. 
	//
	// And I know, I know. It feels like I'm at that usual point - oh, this looks like 
	// a nail, I'll use a hammer - but in the end, is that not just the way computer science 
	// works?
	// 
	// Have I not always said: Everything in computer science comes down to searching?
	//
	
	
	
	//
	// We'll be horribly naive to begin with, but I can already see trouble with this.
	//
	
	//
	// First off, all detections are set as a track that lasts a single frame.
	//
	std::vector< STrack > ptracks;
	for( auto fi0 = frameDetections.begin(); fi0 != frameDetections.end(); ++fi0 )
	{
		for( unsigned pc = 0; pc < fi0->second.size(); ++pc )
		{
			STrack t;
			t.framePeaks[ fi0->first ] = fi0->second[pc];
			t.startFrame = fi0->first;
			t.endFrame   = fi0->first;
			t.merged     = -1;
			
			ptracks.push_back(t);
		}
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
	std::pair<int,int> closestPair(0,1);
#ifdef OCCTRACK_DEBUG
	std::ofstream tmpfi("dists");
#endif
	float sd,td;
	for( unsigned tc0 = 0; tc0 < ptracks.size(); ++tc0 )
	{
		for( unsigned tc1 = tc0+1; tc1 < ptracks.size(); ++tc1 )
		{
			auto p = std::pair<int,int>(tc0,tc1);
			float d = TrackDistance( ptracks[tc0], ptracks[tc1], sd, td );
			
			// we'll limit the number of other tracks that we can link to,
			// we could do this by limiting the difference in time, but here
			// I'm just going to limit the difference between t0 and t1, knowing
			// that tc1 and tc0 are in time order anyway. This'll only fail if we 
			// have too many detections in one frame and fail to span time.
			if( tc1 - tc0 < settings.numNearPeaks )
			{
				distances[p] = d;
				if( distances[p] < distances[ closestPair ] )
				{
					closestPair = p;
				}
				
#ifdef OCCTRACK_DEBUG
				tmpfi << tc0 << " " << tc1 << " : " << distances[p] << " : " << sd << " " << td << endl;
#endif
			}
		}
	}
#ifdef OCCTRACK_DEBUG
	tmpfi.close();
#endif
	
	bool paused, advance;
	paused = advance = false;
	
#ifdef OCCTRACK_DEBUG
	std::ofstream mfi("merges");
#endif
	bool done = false;
	while( !done )
	{
		std::pair<int,int> merge = closestPair;
		
#ifdef OCCTRACK_DEBUG
		dbgImg = cv::Mat( dbgImg.rows, dbgImg.cols, CV_32FC3, cv::Scalar(0,0,0) );
		cout << "tracks: " << ptracks.size() << endl;
		cout << "merge: " << merge.first << " " << merge.second << " : " << distances[ merge ] << endl;
		
		mfi << "---------" << endl;
		mfi << "merge: " << merge.first << " " << merge.second << " : " << distances[ merge ] << endl;
		mfi << "\tsizes: " << ptracks[ merge.first ].framePeaks.size() << " " << ptracks[ merge.second ].framePeaks.size() << endl;
#endif
		
		MergeTracks( merge, ptracks );
		
		//
		// Update the distances and find out what the new closest pair is.
		//
#ifdef OCCTRACK_DEBUG
		cout << "update" << endl;
#endif
		float closestDist = 999999.99;
		std::map< std::pair<int,int>, float > newDistances;
		for( auto pi = distances.begin(); pi != distances.end(); ++pi )
		{
			// for a track pair (a,b)
			// we assert that 'b' is always merged into 'a'.
			// find any distance pairs containing a or b and update them
			// against the new 'a'
			std::pair<int,int> p = pi->first;
			if( p == merge )
			{
				// pair wont exist anymore, do nothing.
			}
			else if( p.first == merge.first)
			{
				// update distance.
				newDistances[ p ] = TrackDistance( ptracks[ p.first ], ptracks[ p.second ], sd, td );
#ifdef OCCTRACK_DEBUG
				mfi << p.first << ", " << p.second << " -> " << newDistances[ p ] << "(" << sd << ", " << td << ") " << endl;
#endif
			}
			else if( p.second == merge.first)
			{
				// update distance.
				newDistances[ p ] = TrackDistance( ptracks[ p.first ], ptracks[ p.second ], sd ,td );
#ifdef OCCTRACK_DEBUG
				mfi << p.first << ", " << p.second << " -> " << newDistances[ p ] << "(" << sd << ", " << td << ") " << endl;
#endif
			}
			else if( p.first == merge.second )
			{
				// so merge.second is no longer an active track.
				// that means that any (merge.second, p.second) distance should become 
				// a (merge.first, p.second) distance. 
				// It's possible we'll come find that distance anyway, but we should make sure.
				// but I'll stick to the rule that (a < b ) the pair (a,b).
				int a = std::min( merge.first, p.second );
				int b = std::max( merge.first, p.second );
				std::pair<int,int> np( a, b );
				newDistances[ np ] = TrackDistance( ptracks[ np.first ], ptracks[ np.second ], sd ,td );
				
#ifdef OCCTRACK_DEBUG
				mfi << np.first << ", " << np.second << " -> " << np.first << ", " << np.second << " : " << newDistances[ np ] << "(" << sd << ", " << td << ") " << endl;
#endif
				
				if( newDistances[ np ] < closestDist )
				{
					closestPair = np;
					closestDist = newDistances[ np ];
				}
			}
			else if( p.second == merge.second )
			{
				// again, merge.second is no longer an active track.
				// that means that any (p.first, merge.second) distance should become 
				// a (p.first, merge.first ).
				// It's possible we'll come find that distance anyway, but we should make sure.
				// but I'll stick to the rule that (a < b ) the pair (a,b).
				int a = std::min( merge.first, p.first );
				int b = std::max( merge.first, p.first );
				std::pair<int,int> np( a, b );
				newDistances[ np ] = TrackDistance( ptracks[ np.first ], ptracks[ np.second ], sd ,td );
				
#ifdef OCCTRACK_DEBUG
				mfi << np.first << ", " << np.second << " -> " << np.first << ", " << np.second << " : " << newDistances[ np ] << "(" << sd << ", " << td << ") " << endl;
#endif
				
				if( newDistances[ np ] < closestDist )
				{
					closestPair = np;
					closestDist = newDistances[ np ];
				}
			}
			else
			{
				newDistances[ p ] = pi->second;
			}
			
			
			
			if( newDistances.find(p) != newDistances.end() && newDistances[ p ] < closestDist )
			{
				closestPair = p;
				closestDist = newDistances[ p ];
			}
		}
		
		assert( newDistances.find( merge ) == newDistances.end() );
		
#ifdef OCCTRACK_DEBUG
		mfi << "next merge: " << closestPair.first << " " << closestPair.second << " : " << distances[ closestPair ] << " " << newDistances[ closestPair ] << endl;
		mfi << "\t" << ptracks[ closestPair.first ].startFrame << " " << ptracks[ closestPair.first ].endFrame << endl;
		mfi << "\t" << ptracks[ closestPair.second ].startFrame << " " << ptracks[ closestPair.second ].endFrame << endl;
#endif
		
		distances = newDistances;
		
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
