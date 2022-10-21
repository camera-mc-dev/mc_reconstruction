#include "imgio/imagesource.h"
#include "imgio/vidsrc.h"
#include "imgio/loadsave.h"
#include "math/mathTypes.h"
#include "math/intersections.h"
#include "math/matrixGenerators.h"

#include "renderer2/basicRenderer.h"
#include "renderer2/geomTools.h"
#include "renderer2/sdfText.h"
#include "renderer2/showImage.h"

#include "misc/c3d.h"

#include <map>
#include <iostream>
#include <fstream>
#include <iomanip>
using std::cout;
using std::endl;

#include "libconfig.h++"

#include "commonConfig/commonConfig.h"

struct SData
{
	std::string dataRoot;
	std::string testRoot;
	
	std::vector<std::string> imgDirs;
	std::vector<std::string> vidFiles;
	std::vector<std::string> calibFiles;
	std::map<std::string,ImageSource*> sources;
	std::map<std::string,unsigned> camKey2Indx;
	
	
	std::vector< std::string > trackFiles;
	std::vector<unsigned> trackStartFrames;
	std::map< std::string, genMatrix > tracks;
	
	
	genMatrix channels;
	int useChannel;
	hVec3D flasherPos;
	
	
	
	bool visualise;
	bool usePrevious;
	int  extraFrameOverlap;
	std::string brightDataFilename;
	
};

void ParseConfig( std::string configFile, SData &data );
void GetSources( SData &data );


class AlignRenderer : public Rendering::BasicRenderer
{
	friend class Rendering::RendererFactory;
	// The constructor should be private or protected so that we are forced 
	// to use the factory...
protected:
	// the constructor creates the renderer with a window of the specified
 	// size, and with the specified title.
	AlignRenderer(unsigned width, unsigned height, std::string title) : BasicRenderer(width,height,title) {}
public:
	bool Step(int &camChange, bool &paused, int &frameChange )
	{
		Render();
		
		win.setActive();
		sf::Event ev;
		while( win.pollEvent(ev) )
		{
			if( ev.type == sf::Event::Closed )
				return false;
			
			if (ev.type == sf::Event::KeyReleased)
			{
				if (ev.key.code == sf::Keyboard::Left )
				{
					camChange = 1;
				}
				if (ev.key.code == sf::Keyboard::Right )
				{
					camChange = -1;
				}
				if (ev.key.code == sf::Keyboard::Space )
				{
					paused = !paused;
				}
				if (ev.key.code == sf::Keyboard::Up )
				{
					frameChange = 1;
				}
				if (ev.key.code == sf::Keyboard::Down )
				{
					frameChange = -1;
				}
			}
		}
		return true;
	}
};




int main(int argc, char* argv[])
{
	if( argc != 2 && argc != 3 && argc != 4)
	{
		cout << "Tool to try and get calibration alignment between camera system and mocap system." << endl;
		cout << "Usage:" << endl;
		cout << argv[0] << " <config file> |< extra overlap frames > <use previous (0 = false,1=true)|" << endl;
		exit(0);
	}
	
	SData data;
	ParseConfig( argv[1], data );
	GetSources( data );
	
	if( argc >= 3 )
	{
		data.extraFrameOverlap = atoi( argv[2] );
	}
	else
	{
		data.extraFrameOverlap = 300;
	}
	
	if( argc == 4 )
	{
		data.usePrevious = atoi( argv[3] ) == 1;
	}
	else
	{
		data.usePrevious = false;
	}
	
	//
	// Create renderer
	//
	CommonConfig ccfg;
	
	// how big are the images?
	cv::Mat img = data.sources.begin()->second->GetCurrent();
	float ar = img.rows / (float) img.cols;

	// create the renderer for display purposes.
	cout << "creating window" << endl;
	std::shared_ptr<AlignRenderer> ren;
	float winW = ccfg.maxSingleWindowWidth;
	float winH = winW * ar;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	if( data.visualise )
		Rendering::RendererFactory::Create( ren, winW, winH, "Calib and mocap alignment check");
	
	
	
	
	
	
	
	
	
	
	//
	// Load the c3d track files.
	//
	data.trackStartFrames.assign( data.trackFiles.size(), 0 );
	for( auto tfc = 0; tfc < data.trackFiles.size(); ++tfc )
	{
		cout << "loading: " << data.trackFiles[tfc] << endl;
		std::map< std::string, genMatrix > newTracks;
		genMatrix newChannels;
		LoadC3DFile( data.trackFiles[tfc], data.trackStartFrames[tfc], newTracks, newChannels );
		cout << "\tnew tracks: " << newTracks.size() << endl;
		for( auto ti = newTracks.begin(); ti != newTracks.end(); ++ti )
		{
			cout << "\t" << ti->first << endl;
			if( data.tracks.find( ti->first ) == data.tracks.end() )
			{
				data.tracks[ ti->first ] = ti->second;
			}
			else
			{
				cout << "Track " << ti->first << " from " << data.trackFiles[tfc] << " already exists from other file?" << endl;
				exit(0);
			}
		}
		
		if( data.channels.cols() > 0 && data.channels.cols() != newChannels.cols() )
		{
			cout << "ignoring channels from file: " << data.trackFiles[tfc] << " because it has different frames from previous channels data " << endl;
			cout << newChannels.cols() << " vs: " << data.channels.cols() << endl;
		}
		else if( data.channels.cols() > 0 )
		{
			genMatrix tmp( data.channels.rows() + newChannels.rows(), data.channels.cols() );
			tmp << data.channels, newChannels;
			
			data.channels = tmp;
		}
		else
			data.channels = newChannels;
	}
	
	cout << "track names: " << endl;
	unsigned long minElements = 99999999;
	for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
	{
		minElements = std::min( minElements, (unsigned long)ti->second.cols() );
		cout << "\t" << ti->first << " " << ti->second.cols() << endl;
	}
	cout << "min elements: " << minElements << endl;
	
	std::string ledName;
	if( data.tracks.find("blinky") != data.tracks.end() )
	{
		ledName = "blinky";
	}
	else if( data.tracks.find("IR_LED") != data.tracks.end() )
	{
		ledName = "IR_LED";
	}
	else
	{
		cout << "Can't recognise LED marker name in .c3d files. Checking if we have analog data.." << endl;
		if( data.useChannel < 0 || data.channels.rows() < data.useChannel )
		{
			cout << "had " << data.channels.rows() << " analog channels" << endl;
			cout << "user asked for channel " << data.useChannel << endl;
			cout << "so, we can't use channel data either." << endl << endl;
			
			cout << "can try -7 guess?" << endl;
			
			// can we try the -7 guess? Only if there are other markers.
			if( data.tracks.empty() )
			{
				std::stringstream ss;
				ss << data.dataRoot << data.testRoot << "/frameOffset";
				cout << "saving offset to: " << ss.str() << endl;
				std::ofstream finalfi(ss.str());
				finalfi << "offset: " << 0 << endl << endl;
				finalfi << "numVidFrames: " << 0 << endl;
				finalfi << "numMocapFrames: " << 0 << endl;
				finalfi << "baseOffset: " << 0 << endl;
				finalfi << "extraOffset: " << 0 << endl << endl << endl;
				finalfi << "--- info ---" << endl;
				finalfi << "mocap frame = video frame + offset" << endl;
				finalfi << "numVidFrames is number of video frames in sequence" << endl;
				finalfi << "numMocapFrames is number of mocap frames in sequence" << endl;
				finalfi << "baseOffset = numMocapFrames - numVidFrames (in theory, both stop at the same time)" << endl;
				finalfi << "extraOffset = offset - baseOffset (there's a chance this might be constant between trials)" << endl;
				finalfi << endl;
				finalfi << "!!! No marker info available for setting offset !!!" << endl;
				finalfi.close();
				
				throw std::runtime_error( "Can't recognise name of LED marker in any loaded .c3d files, and no other marker tracks to guess with. " );
			}
			auto ti = data.tracks.begin();
			
			int numVidFrames = data.sources.begin()->second->GetNumImages();
			int numMocapFrames = data.tracks.begin()->second.cols();
			int baseOffset = numMocapFrames - numVidFrames;
			int extraOffset = -7;
			
			std::stringstream ss;
			ss << data.dataRoot << data.testRoot << "/frameOffset";
			cout << "saving offset to: " << ss.str() << endl;
			std::ofstream finalfi(ss.str());
			finalfi << "offset: " << baseOffset + extraOffset << endl << endl;
			finalfi << "numVidFrames: " << numVidFrames << endl;
			finalfi << "numMocapFrames: " << numMocapFrames << endl;
			finalfi << "baseOffset: " << numMocapFrames - numVidFrames << endl;
			finalfi << "extraOffset: " << extraOffset << endl << endl << endl;
			finalfi << "--- info ---" << endl;
			finalfi << "mocap frame = video frame + offset" << endl;
			finalfi << "numVidFrames is number of video frames in sequence" << endl;
			finalfi << "numMocapFrames is number of mocap frames in sequence" << endl;
			finalfi << "baseOffset = numMocapFrames - numVidFrames (in theory, both stop at the same time)" << endl;
			finalfi << "extraOffset = offset - baseOffset (there's a chance this might be constant between trials)" << endl;
			finalfi << endl;
			finalfi << "!!! Guessed offset from a fairly typical -7 extra offset !!!" << endl;
			finalfi.close();
			
			throw std::runtime_error( "Can't recognise name of LED marker in any loaded .c3d files, and no other marker tracks to guess with. " );
			
		}
	}
	
	
// 	std::ofstream tst("chtst");
// 	tst << data.channels << endl << endl;
	
	
	
	//
	// Search for best time alignment.
	//
	
	
	//
	// Find out the basic non-origin location of the "blinky" marker.
	//
	hVec3D blinky;
	if( data.useChannel < 0 )
	{
		std::vector<float> xs, ys, zs;
		for( unsigned fc = 0; fc < data.tracks[ledName].cols(); ++fc )
		{
			hVec3D b = data.tracks[ledName].col(fc).head(4);
			if( b(0) != 0 && b(1) != 0 && b(2) != 0 )
			{
				xs.push_back( b(0) );
				ys.push_back( b(1) );
				zs.push_back( b(2) );
			}
		}
		std::sort( xs.begin(), xs.end() );
		std::sort( ys.begin(), ys.end() );
		std::sort( zs.begin(), zs.end() );
		

		blinky << xs[ xs.size()/2 ], ys[ys.size()/2], zs[zs.size()/2], 1.0f;
		cout << "blinky mean position: " << blinky.transpose() << endl;
	}
	else
	{
		blinky = data.flasherPos;
	}
	
	//
	// OK, it will be a very subtle signal, but here's the plan.
	//
	// for each frame:
	// 1) Project the 'blinky' marker into the image.
	// 2) compute the mean brightness of the image region around that marker - say a small 32x32 window.
	// 
	// amplify the signal to make it clearer, but we should see that when the light comes on
	// the mean brightness in most/all of the images increases, and then drops when the light goes off.
	//
	
	bool done = false;
	int fc = 0;
	std::map< int, std::string > ind2id;
	std::map< std::string, int > id2ind;
	int cind = 0;
	for( auto ci = data.sources.begin(); ci != data.sources.end(); ++ci )
	{
		ind2id[cind] = ci->first;
		id2ind[ ci->first ] = cind;
		++cind;
	}
	
	cind = 0;
	bool paused = false;
	int camChange = 0;
	int frameAdvance = 0;
	int fc0 = 0;
	std::vector< std::vector<float> > meanBrights;
	genMatrix brightData;
	if( !data.usePrevious )
	{
		while(!done)
		{
			//
			// For each image...
			//
			std::vector<float> brights( data.sources.size(), 0 );
			for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
			{
				Calibration &calib = sidi->second->GetCalibration();
				cv::Mat img = sidi->second->GetCurrent();
				
				// project the blinking LED into the image.
				hVec2D bcp = calib.Project( blinky );
				
				// get a small window around that region - 32 by 32 pixels should do it.
				// compute mean brightness of region.
				float bright = 0.0f;
				for( unsigned rc = bcp(1) - 16; rc < bcp(1) + 16; ++rc )
				{
					for( unsigned cc = bcp(0) - 16; cc < bcp(0) + 16; ++cc )
					{
						cv::Vec3b &p = img.at<cv::Vec3b>(rc,cc);
						
						bright += (p[0] + p[1] + p[2]) / 3.0f;
						
					}
				}
				bright /= 32*32;
				
				brights[ id2ind[ sidi->first ] ] = bright;
			}
			meanBrights.push_back( brights );
			
			//
			// Draw the blinking LED.
			//
			if( data.visualise )
			{
				cv::Mat img = data.sources[ ind2id[cind] ]->GetCurrent();
				Calibration &calib = data.sources[ ind2id[cind] ]->GetCalibration();
				
				ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
				ren->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
				
				hVec2D bcp = calib.Project( blinky );
				cv::circle( img, cv::Point( bcp(0), bcp(1) ), 15, cv::Scalar(255,255,0), 2 );
				
				
				ren->SetBGImage(img);
				
				frameAdvance = 0;
				done = !ren->Step(camChange, paused, frameAdvance);
				while( paused && frameAdvance == 0 )
				{
					done = !ren->Step(camChange, paused, frameAdvance);
				}
				if( !paused || (paused && frameAdvance == 1 ) )
				{
					for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
						done = done || !sidi->second->Advance();
				}
				
				if( camChange != 0 )
				{
					if( camChange > 0 && cind < ind2id.size()-1 )
						++cind;
					else if( camChange < 0 && cind > 0 )
						--cind;
					camChange = 0;
				}
			}
			else
			{
				for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
					done = done || !sidi->second->Advance();
			}
			
			++fc0;
			
		}
		
		
		//
		// Now that we have the per-frame data, let us make it easier to deal with.
		//
		brightData = genMatrix::Zero( meanBrights[0].size(), meanBrights.size() );
		cout << "bd shape: " << brightData.rows() << " " << brightData.cols() << endl;
		for( unsigned fc = 0; fc < meanBrights.size(); ++fc )
		{
			for( unsigned sc = 0; sc < meanBrights[fc].size(); ++sc )
			{
				brightData(sc,fc) = meanBrights[fc][sc];
			}
		}
		
		//
		// each row of that matrix is the brightness data for a camera view. Find the max and min on each row and normalise
		// max and min can lead to insufficient contrast, so we now use max against median.
		//
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			float m = brightData.row(rc).minCoeff();
			float M = brightData.row(rc).maxCoeff();
			
			std::vector<float> rowVals( brightData.cols() );
			Eigen::Map< Eigen::Matrix<float, 1, Eigen::Dynamic> > (&rowVals[0], 1, brightData.cols()) = brightData.row(rc);
			std::sort( rowVals.begin(), rowVals.end() );
			float md = rowVals[ rowVals.size()/2 ];
			float tq = rowVals[ 0.75*rowVals.size() ];
			brightData.row(rc).array() -= md;
			brightData.row(rc).array() /= (tq-md);
			for( unsigned cc = 0; cc < brightData.cols(); ++cc )
			{
				brightData(rc,cc) = std::min(1.0f, brightData(rc,cc) );
				brightData(rc,cc) = std::max(0.0f, brightData(rc,cc) );
			}
			
			cout << rc << "m, md, tq, M: " << m << " " << md << " " << tq << " " << M << endl;
			
		}
		
		std::ofstream tmp( data.brightDataFilename );
		tmp << brightData.rows() << " " << brightData.cols() << endl;
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < brightData.cols(); ++cc )
			{
				tmp << brightData(rc,cc) << " ";
			}
			tmp << endl;
		}
	}
	else
	{
		std::ifstream tmp( data.brightDataFilename );
		int nr, nc;
		tmp >> nr;
		tmp >> nc;
		brightData = genMatrix::Zero( nr, nc );
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < brightData.cols(); ++cc )
			{
				tmp >> brightData(rc,cc);
			}
		}
	}
	
	//
	// Let us visualise that, but with a little bit of luck, that'll show us our timing signal?
	//
	cv::Mat bd( brightData.rows()*10, brightData.cols(), CV_32FC1, cv::Scalar(0.0f) );
	cout << "bd cv shape: " << bd.rows << " " << bd.cols << endl;
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 10; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = brightData(rc,cc);
			}
		}
	}
	if( data.visualise )
		Rendering::ShowImage(bd, 1500);
	
	//
	// We don't know how long each LED is on for, so how shall we find the frames where
	// the light switches on?
	//
	// Convolution of the light signal with a simple filter should give us an answer...
	//
	Eigen::VectorXf filter(30);
	for( unsigned cc = 0; cc < filter.rows(); ++cc )
	{
		if( cc < filter.rows()/2 ) filter(cc) = -1.0f;
		else filter(cc) = 1.0f;
	}
	genMatrix bd2 = genMatrix::Zero( brightData.rows(), brightData.cols() );
	for( unsigned cc = 0; cc < brightData.cols() - filter.rows(); ++cc )
	{
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			Eigen::VectorXf w = brightData.block(rc,cc, 1, filter.rows()).transpose();
			bd2(rc,cc+filter.rows()/2) = (w.array() * filter.array()).sum() / (float)filter.rows();  // cc+filter.rows()/2 so that signal peaks on the event
		}
	}
	
	bd = cv::Mat( brightData.rows()*10, brightData.cols(), CV_32FC1, cv::Scalar(0.0f) );
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 10; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = bd2(rc,cc);
			}
		}
	}
	if( data.visualise )
		Rendering::ShowImage(bd, 1500);
	
	
	//
	// If we gaussian filter across time using all the views, we can create cross-view consistency (?)
	// 
	Eigen::VectorXf gfilter(60);
	genMatrix bd2g = bd2;
	
	Eigen::VectorXf gscv;
	float numRpt = 3.0f;
	for(unsigned rpt = 0; rpt < numRpt; ++rpt )
	{
		// create the gaussian filter.
		for( unsigned cc = 0; cc < 60; ++cc )
		{
			float x = cc - 30.0;
			float sig = 5.0f; //(11-rpt);
			gfilter(cc) = exp( -(x*x)/(sig*sig)  );
			cout << std::setw(6) << std::fixed << std::setprecision(2) << gfilter(cc);
		}
		float gsc = gfilter.array().sum();
		
		// Apply the gaussian filter to get our 1D response
		gscv = Eigen::VectorXf::Zero(brightData.cols());
		for( unsigned cc = 0; cc < brightData.cols() - 60; ++cc )
		{
			for( unsigned rc = 0; rc < brightData.rows(); ++rc )
			{
				Eigen::VectorXf w = bd2g.block(rc,cc, 1, 60).transpose();
				gscv(cc+30) += (w.array() * gfilter.array()).sum()/gsc;
			}
			gscv(cc+30) /= brightData.rows();
		}
		
		
		//
		// Then push each viewpoint towards that filtered mean
		//
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < brightData.cols(); ++cc )
			{
				// too lazy to do this properly.
				bd2g(rc,cc) = (numRpt-1.0)/numRpt * bd2g(rc,cc)  + 1.0/numRpt * gscv(cc);
			}
// 			bd2g.row(rc) =  * bd2g.row(rc) + 1.0/numRpt * gscv;
		}
		
		

		
	}
	

	if( data.visualise )
	{
		std::ofstream tstfi1("bd2g-tst");
		for( unsigned rc = 0; rc < bd2g.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd2g.cols(); ++cc )
			{
				tstfi1 << bd2g(rc,cc) << " ";
			}
			tstfi1 << endl;
		}
		tstfi1 << endl;
		Rendering::ShowImage(bd, 1500);
	}
	
	
	
	
	
	
	
	
	//
	// Now we find the brightness peaks. They should be strong enough that we can make use of a threshold
	// as well as a local peak filter.
	//
	genMatrix bd3 = genMatrix::Zero( brightData.rows(), brightData.cols() );
	float thr = (bd2g.mean() + bd2g.maxCoeff()) / 2.0f;
	cout << "bd2g m, mn, M: " << bd2g.minCoeff() << " " << bd2g.mean() << " " << bd2g.maxCoeff() << endl;
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 20; cc < brightData.cols() - 20; ++cc )
		{
			Eigen::VectorXf w = bd2g.block(rc,cc-10, 1, 20).transpose();
			if(
			    bd2g(rc,cc) > thr            && 
			    bd2g(rc,cc) > bd2g(rc,cc-1) &&
			    bd2g(rc,cc) > bd2g(rc,cc+1) 
			  )
			{
				bd3(rc,cc) = 1.0f;
			}
		}
	}
	cout << "bd3 max: " << bd3.maxCoeff() << endl;
	
	bd = cv::Mat( brightData.rows()*10, brightData.cols(), CV_32FC1, cv::Scalar(0.0f) );
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 10; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = bd3(rc,cc);
			}
		}
	}
	if( data.visualise )
	{
		std::ofstream tstfi1("bd3-tst");
		for( unsigned rc = 0; rc < bd3.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd3.cols(); ++cc )
			{
				tstfi1 << bd3(rc,cc) << " ";
			}
			tstfi1 << endl;
		}
		tstfi1.close();
		Rendering::ShowImage(bd, 1500);
	}
	
	//
	// Now we use the multiple cameras to get robustness with a simple count.
	//
	genMatrix bd4 = genMatrix::Zero( brightData.rows(), brightData.cols() );
	for( unsigned cc = 0; cc < brightData.cols(); ++cc )
	{
		int c = 0;
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			if( bd3(rc,cc) == 1.0f )
				++c;
		}
		if( c > brightData.rows() / 2 )
		{
			for( unsigned rc = 0; rc < brightData.rows(); ++rc )
			{
				bd4(rc,cc) = 1.0f;
			}
		}
	}
	cout << "bd4 max: " << bd4.maxCoeff() << endl;
	
	bd = cv::Mat( brightData.rows()*10, brightData.cols(), CV_32FC1, cv::Scalar(0.0f) );
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 10; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = bd4(rc,cc);
			}
		}
	}
	if( data.visualise )
		Rendering::ShowImage(bd, 1500);
	
	{
		std::ofstream outfi("bd1.dat");
		outfi << brightData.rows() << " " << brightData.cols() << endl;
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < brightData.cols(); ++cc )
			{
				outfi << brightData(rc,cc) << " ";
			}
			outfi << endl;
		}
	}
	{
		std::ofstream outfi("bd2.dat");
		outfi << bd2.rows() << " " << bd2.cols() << endl;
		for( unsigned rc = 0; rc < bd2.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd2.cols(); ++cc )
			{
				outfi << bd2(rc,cc) << " ";
			}
			outfi << endl;
		}
	}
	{
		std::ofstream outfi("bd3.dat");
		outfi << bd3.rows() << " " << bd3.cols() << endl;
		for( unsigned rc = 0; rc < bd3.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd3.cols(); ++cc )
			{
				outfi << bd3(rc,cc) << " ";
			}
			outfi << endl;
		}
	}
	{
		std::ofstream outfi("bd4.dat");
		outfi << bd4.rows() << " " << bd4.cols() << endl;
		for( unsigned rc = 0; rc < bd4.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd4.cols(); ++cc )
			{
				outfi << bd4(rc,cc) << " ";
			}
			outfi << endl;
		}
	}
	
	//
	// And just like that, we have found the start of our flashes (assuming I lined up my convolutions correctly)
	// except - I think I'm finding the _ends_ of my flashes too. Do I want that?
	//
	std::vector<float> flashStarts;
	for( unsigned cc = 0; cc < bd4.cols(); ++cc )
	{
		if( bd4(0,cc) == 1.0f )
			flashStarts.push_back(cc);
	}
	
	
	//
	// We know that the video _should_ align to the last n frames of the mocap data,
	// where n is the number of video frames, which we know from the columns of bd.
	//
	int numVidFrames = brightData.cols();
	int numMocapFrames = data.tracks.begin()->second.cols();
	cout << numVidFrames << " " << numMocapFrames << endl;
	int baseOffset = numMocapFrames - numVidFrames;
	
	
	//
	// So now we just have to set about figuring out an offset that aligns our 
	// detected light flashes with the changing state of the "blinky" track.
	//
	// We can reasonably expect the extra time alignment to be in the range -40 to 40
	//
	
	// find the mocap blinky starts.
	std::vector<float> blinkyStarts;
	if( data.useChannel < 0 )
	{
		for( unsigned cc = 0; cc < data.tracks[ledName].cols()-1; ++cc )
		{
			Eigen::VectorXf c0 = data.tracks[ledName].col(cc);
			Eigen::VectorXf c1 = data.tracks[ledName].col(cc+1);
			
			if( c0(0) == 0.0f && c0(1) == 0.0f && c0(2) == 0.0f &&
			(c1(0) != 0.0f && c1(1) != 0.0f && c1(2) != 0.0f)  )
			{
				blinkyStarts.push_back(cc);
			}
		}
	}
	else
	{
		// what is the mean value of the channel?
		// I _think_ the channel goes -ve for on, near 0 for off.
		Eigen::VectorXf blinkData = data.channels.row( data.useChannel );
		float mn = blinkData.mean();
		
		cout << "blinkData m, mn, M: " << blinkData.minCoeff() << " " << blinkData.mean() << " " << blinkData.maxCoeff() << endl;
		std::ofstream tst("chtst");
		for( unsigned cc = 0; cc < data.channels.cols()-1; ++cc )
		{
			float v0 = data.channels( data.useChannel, cc   );
			float v1 = data.channels( data.useChannel, cc+1 );
			tst << cc << " : " << std::setw(8) << v0 << " " << std::setw(8) << v1;
			if( v0 > mn && v1 < mn )
			{
				blinkyStarts.push_back(cc);
				tst << " <---";
			}
			tst  << endl;
		}
		
		// there's a further complication in that the analog signal is 
		// at a faster framerate than the video.
		float sc = data.channels.cols() / minElements;
		for( unsigned bsc = 0; bsc < blinkyStarts.size(); ++bsc )
		{
			blinkyStarts[bsc] /= sc;
		}
	}
	
	
	cout << "blink marker starts:" << endl;
	for( unsigned i = 0; i < blinkyStarts.size(); ++i )
	{
		cout << blinkyStarts[i] << " ";
	}
	cout << endl;
	
	cout << "flash starts: " << endl;
	for( unsigned i = 0; i < flashStarts.size(); ++i )
	{
		cout << flashStarts[i] << " ";
	}
	cout << endl;
	
	cout << "blink marker periods:" << endl;
	for( unsigned i = 0; i < blinkyStarts.size()-1; ++i )
	{
		cout << blinkyStarts[i+1] - blinkyStarts[i]<< " ";
	}
	cout << endl;
	
	cout << "flash periods: " << endl;
	for( unsigned i = 0; i < flashStarts.size()-1; ++i )
	{
		cout << flashStarts[i+1] - flashStarts[i] << " ";
	}
	cout << endl;
	
	
	
	//
	// Now we need to slide the shorter signal over the longer signal like a correlation and find the best alignment
	//
	
	// which set of flahses is shorter?
	std::vector<float> *inner, *outer;
	if( blinkyStarts.size() < flashStarts.size() )
	{
		inner = &blinkyStarts;
		outer = &flashStarts;
	}
	else
	{
		inner = &flashStarts;
		outer = &blinkyStarts;
	}
	
	
	float bestErr = 99999999.99f;
	int bestInd = 0;
	
	
	// put the shorter (inner) sequence at the start of the longer (outer) sequence, then at the 1st, position, then 2nd.. etc..
	//
	// Ah! would'st that 'twer so simple. What prey tell, oh Murray of coding days past, shall be done 't resolve that woderful
	// scenario where both are equal, but we pick the wrong inner, and, oh dear! That inner needed to move _before_ the outer?
	// What then, dear chappy dear lad, dear ancester of shared bones? 
	//
	// Seriously though... can there ever in practice be a situation where a shorter inner needs to move before the outer? Ach, 
	// "maybe" is the best answer to that, so we need a fix that allows for that.
	//
	// Interesting, sometimes the mocap LED can produce multiple flashes for one flash which I think causes us to need to allow
	// the inner to slide beyond the outer at the end as well as at the start.
	std::vector<int> offsets;
	int i = -2;
	done = false;
	while( !done )
	{
		//
		// Get each flash-start of the inner sequence as a time from the "first" flash.
		// Note that, to allow for the inner sequence to start _before_ the outer sequence
		// we might that "first" flash to be several flashes inside the inner sequence.
		//
		std::vector<int> inner0( inner->size() );
		for( unsigned c = 0; c < inner0.size(); ++c )
		{
			inner0[c] = inner->at(c) - inner->at( std::abs( std::min(0, i)) );
		}
		
		//
		// The duration of the inner sequence is now inner0.back() ( i.e. inner.back() - inner[ first ] )
		// where first may be 0,1,2 depending on how much of the head we allow to crop off to allow inner 
		// to start before outer.
		//
		std::vector<int> outer0;
		int d = 0;
		unsigned c = std::max(i,0);
		while( d < inner0.back() + 10 && c < outer->size() )
		{
		
			d = outer->at(c) - outer->at(std::max(i,0));
			outer0.push_back( d );
			++c;
		}
		
		cout << "---- " << i << " ---- " << endl;
		
		cout << "inner0 (" << inner0.size() << "): ";
		for( unsigned c = 0; c < inner0.size(); ++c )
		{
			cout << inner0[c] << " ";
		}
		cout << endl;
		
		cout << "outer0 (" << outer0.size() << "): ";
		for( unsigned c = 0; c < outer0.size(); ++c )
		{
			cout << outer0[c] << " ";
		}
		cout << endl;
		
		cout << "inner0 per: ";
		for( unsigned c = 0; c < inner0.size()-1; ++c )
		{
			cout << inner0[c+1] - inner0[c] << " ";
		}
		cout << endl;
		
		cout << "outer0 per: ";
		for( unsigned c = 0; c < outer0.size()-1; ++c )
		{
			cout << outer0[c+1] - outer0[c] << " ";
		}
		cout << endl;
		
		// find the mutually best inner for each outer.
		genMatrix E = genMatrix::Zero( inner0.size(), outer0.size() );
		for( unsigned ic = 0; ic < inner0.size(); ++ic )
		{
			for( unsigned oc = 0; oc < outer0.size(); ++oc )
			{
				E( ic, oc ) = abs( inner0[ic] - outer0[oc] );
			}
		}
		
// 		cout << E << endl;
		std::vector< int > i2o( inner0.size(), -1 );
		int ic, oc;
		float x = E.minCoeff(&ic, &oc);
		while( x < 90000 )
		{
			i2o[ic] = oc;
			for( unsigned oc2 = 0; oc2 < E.cols(); ++oc2 )
				E(ic, oc2) = 90000;
			for( unsigned ic2 = 0; ic2 < E.rows(); ++ic2 )
				E(ic2, oc) = 90000;
			x = E.minCoeff(&ic, &oc);
		}
		
		
		
		// find the offsets implied by this location.
		std::vector<int> curOffsets;
		for( unsigned ic = 0; ic < i2o.size(); ++ic )
		{
			if( i2o[ic] >= 0 )
			{
				int offset = outer->at( i2o[ic] + std::max(i,0) ) - inner->at(ic);
				cout << i2o[ic] << " " << inner0[ic] << " " << outer0[ i2o[ic] ] << "  off: " << offset << endl;
				curOffsets.push_back( offset );
			}
			else
			{
				cout << i2o[ic] << " " << inner0[ic] << " " << outer0[ i2o[ic] ] << "  N/A" << endl;
			}
		}
		
		// find the median offset.
		std::vector<int> cof2 = curOffsets;
		std::sort( cof2.begin(), cof2.end() );
		int medianOffset = cof2[ cof2.size()/2 ];
		
		// compute the errors
		std::vector<int> errs;
		for( unsigned oc = 0; oc < curOffsets.size(); ++oc )
		{
			int e = abs( curOffsets[oc] - medianOffset );
			cout << curOffsets[oc] << " -> " << e << endl;
			errs.push_back(e);
		}
		
		
		if( errs.size() >= 2 )
		{
			// we want a somewhat robust error - is median enough?
			std::sort( errs.begin(), errs.end() );
			float safeMedianErr = errs[ 0.75 * errs.size() ]; 
			
			if( safeMedianErr <= bestErr+2 ) // <= because we're looking for the last best alignment (so accept a slightly worse error if needs be)
			{
				bestErr = safeMedianErr;
				bestInd = i;
				offsets = curOffsets;
			}
			cout << safeMedianErr << " (" << bestErr << ") " << endl << endl << endl;
		}
		
		
		++i;
		done = ( inner0.back() - outer0.back() ) > data.extraFrameOverlap;
	}
	
	
	cout << bestInd << " " << bestErr << " " << outer->at( std::max(0,bestInd) ) << " " << inner->at( std::abs( std::min(0, bestInd) ) ) << endl;
	
	//
	// So, based on that, the final offset must be...
	//
	
	std::vector< int > off2( offsets.begin() + offsets.size()/2, offsets.end() );
	
	// action tends to be latter half of data, so we'll always use the median of 
	// the latter half of the data...
	
	std::sort( off2.begin(), off2.end() );
	int finalOffset = off2[ off2.size()/2];

	// final offset should be mocframe = vidframe+offset
	// currently it is outerFrame = innerFrame+offset
	if( inner == &blinkyStarts )
	{
		finalOffset = -finalOffset;
	}
	cout << "final offset: " << finalOffset << endl;
	
	
	std::stringstream ss;
	ss << data.dataRoot << data.testRoot << "/frameOffset";
	cout << "saving offset to: " << ss.str() << endl;
	std::ofstream finalfi(ss.str());
	finalfi << "offset: " << finalOffset << endl << endl;
	finalfi << "numVidFrames: " << numVidFrames << endl;
	finalfi << "numMocapFrames: " << numMocapFrames << endl;
	finalfi << "baseOffset: " << numMocapFrames - numVidFrames << endl;
	finalfi << "extraOffset: " << finalOffset - baseOffset << endl << endl << endl;
	finalfi << "--- info ---" << endl;
	finalfi << "mocap frame = video frame + offset" << endl;
	finalfi << "numVidFrames is number of video frames in sequence" << endl;
	finalfi << "numMocapFrames is number of mocap frames in sequence" << endl;
	finalfi << "baseOffset = numMocapFrames - numVidFrames (in theory, both stop at the same time)" << endl;
	finalfi << "extraOffset = offset - baseOffset (there's a chance this might be constant between trials)" << endl;
	
	
	
	if( data.visualise )
	{
		//
		// Final render
		//
		cout << "Jumping to frame 0..." << endl;
	// 	for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
	// 		data.sources[ ind2id[cind] ]->JumpToFrame(0);
		data.sources.clear();
		GetSources(data);
		
		
		done = false;
		fc = 0;

		
		cind = 0;
		paused = false;
		camChange = 0;
		frameAdvance = 0;
		fc0 = 0;
		cv::Mat img0;
		while(!done)
		{
			int fc = data.sources[ ind2id[cind] ]->GetCurrentFrameID();
			cout << fc << " " << fc0 << endl;
			
			
			Calibration &calib = data.sources[ ind2id[cind] ]->GetCalibration();
			img = data.sources[ ind2id[cind] ]->GetCurrent().clone();
			
			ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			ren->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
			
			//
			// Draw whole tracks faintly.
			//
// 			cv::Mat imgCpy = img.clone();
			
			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
			{
// 				for( unsigned fc = 0; fc + finalOffset < ti->second.cols()-1; ++fc )
// 				{
// 					hVec2D a = calib.Project( ti->second.col( fc + finalOffset ).head(4) );
// 					hVec2D b = calib.Project( ti->second.col( fc+1 + finalOffset ).head(4) );
// 					
// 					cv::line( imgCpy, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), cv::Scalar(0,255,0), 2);
// 				}
				
// 				cv::Mat tmp;
// 				cv::addWeighted(img, 0.7, imgCpy, 0.3, 0.0, tmp );
// 				img = tmp;
				
				if( fc-2 + finalOffset < 0 || fc+2 + finalOffset >=  ti->second.cols() )
					continue;
			
				hVec2D a = calib.Project( ti->second.col( fc-2 + finalOffset ).head(4) );
				hVec2D b = calib.Project( ti->second.col( fc-1 + finalOffset ).head(4) );
				hVec2D c = calib.Project( ti->second.col( fc   + finalOffset ).head(4) );
				hVec2D d = calib.Project( ti->second.col( fc+1 + finalOffset ).head(4) );
				hVec2D e = calib.Project( ti->second.col( fc+2 + finalOffset ).head(4) );
				
				cv::line( img, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), cv::Scalar(128,  0,   0), 2);
				cv::line( img, cv::Point( b(0), b(1) ), cv::Point( c(0), c(1) ), cv::Scalar(255,  0,   0), 2);
				cv::line( img, cv::Point( c(0), c(1) ), cv::Point( d(0), d(1) ), cv::Scalar(  0,  0, 255), 2);
				cv::line( img, cv::Point( d(0), d(1) ), cv::Point( e(0), e(1) ), cv::Scalar(  0,  0, 128), 2);
			}
			
			
// 			//
// 			// Draw all markers
// 			//
// 			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
// 			{
// 				hVec2D bcp = calib.Project( ti->second.col( fc + finalOffset ).head(4) );
// 				cv::circle( img, cv::Point( bcp(0), bcp(1) ), 15, cv::Scalar(0,255,0), 2 );
// 				
// 			}
			
			
			//
			// Draw the blinking LED.
			//
			
			if( fc + finalOffset >= 0 && fc+finalOffset < data.tracks[ledName].cols() )
			{
				hVec2D bcp = calib.Project( data.tracks[ledName].col( fc + finalOffset ).head(4) );
				cv::circle( img, cv::Point( bcp(0), bcp(1) ), 15, cv::Scalar(255,255,0), 2 );
			}
			
			ren->SetBGImage(img);
			
			frameAdvance = 0;
			done = !ren->Step(camChange, paused, frameAdvance);
			while( paused && frameAdvance == 0 )
			{
				done = !ren->Step(camChange, paused, frameAdvance);
			}
			cout << "cc, fa: " << camChange << " " << frameAdvance << " " << cind << endl;
			if( !paused || (paused && frameAdvance == 1 ) )
				for( auto sidi = data.sources.begin(); sidi != data.sources.end(); ++sidi )
					done = done || !sidi->second->Advance();
			
			if( camChange != 0 )
			{
				if( camChange > 0 && cind < ind2id.size()-1 )
					++cind;
				else if( camChange < 0 && cind > 0 )
					--cind;
				camChange = 0;
			}
			
			
			//
			// Save image
			//
			std::stringstream fnss;
			fnss << "alignVid/" << std::setw(6) << std::setfill('0') << fc0 << ".jpg";
			cv::Mat grab = ren->Capture();
			SaveImage( grab, fnss.str() );
			
			++fc0;
			
		}
	}
	
	
}




void ParseConfig( std::string configFile, SData &data )
{
	CommonConfig ccfg;
	
	data.dataRoot = ccfg.dataRoot;
	data.visualise = false;
	
	try
	{
		libconfig::Config cfg;
		cout << "read : " << configFile.c_str() << endl;
		cfg.readFile(configFile.c_str() );
		
		if( cfg.exists("dataRoot") )
			data.dataRoot = (const char*)cfg.lookup("dataRoot");
		data.testRoot = (const char*)cfg.lookup("testRoot");
		
		if( cfg.exists("imgDirs" ) )
		{
			libconfig::Setting &idirs = cfg.lookup("imgDirs");
			for( unsigned ic = 0; ic < idirs.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) idirs[ic];
				data.imgDirs.push_back(s);
			}
		}
		
		if( cfg.exists("vidFiles" ) )
		{
			libconfig::Setting &vidfs = cfg.lookup("vidFiles");
			for( unsigned ic = 0; ic < vidfs.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) vidfs[ic];
				data.vidFiles.push_back(s);
			}
			
			libconfig::Setting &cfiles = cfg.lookup("calibFiles");
			for( unsigned ic = 0; ic < cfiles.getLength(); ++ic )
			{
				std::string s;
				s = data.dataRoot + data.testRoot + (const char*) cfiles[ic];
				data.calibFiles.push_back(s);
			}
		}
		
		
		libconfig::Setting &trkfs = cfg.lookup("trackFiles");
		for( unsigned tfc = 0; tfc < trkfs.getLength(); ++tfc )
		{
			std::stringstream ss;
			ss << data.dataRoot << "/" << data.testRoot << "/" << (const char*) trkfs[tfc];
			data.trackFiles.push_back( ss.str() );
		}
		
		data.useChannel = -1;
		if( cfg.exists("useChannel") )
		{
			data.useChannel = cfg.lookup("useChannel");
			libconfig::Setting &flasherPosSetting = cfg.lookup("flasherPos");
			assert( flasherPosSetting.getLength() == 4 );
			data.flasherPos << flasherPosSetting[0], flasherPosSetting[1], flasherPosSetting[2], flasherPosSetting[3];
		}
		
		if( cfg.exists("visualise") )
		{
			data.visualise = cfg.lookup("visualise");
		}
		
	}
	catch( libconfig::SettingException &e)
	{
		cout << "Setting error: " << endl;
		cout << e.what() << endl;
		cout << e.getPath() << endl;
		exit(0);
	}
	catch( libconfig::ParseException &e )
	{
		cout << "Parse error:" << endl;
		cout << e.what() << endl;
		cout << e.getError() << endl;
		cout << e.getFile() << endl;
		cout << e.getLine() << endl;
		exit(0);
	}
	
	std::stringstream ss;
	ss << data.dataRoot << "/" << data.testRoot << "/bright.data";
	data.brightDataFilename = ss.str();
}


void GetSources( SData &data )
{
	std::string camKey;
	if( data.imgDirs.size() > 0 )
	{
		for( unsigned ic = 0; ic < data.imgDirs.size(); ++ic )
		{
			int a = data.imgDirs[ic].rfind( "/", data.imgDirs[ic].size()-2 );
			int b = data.imgDirs[ic].find( "/", a+1 );
			
			camKey = std::string( data.imgDirs[ic].begin() + a+1, data.imgDirs[ic].begin() + b );
			
			cout << "creating source: " << data.imgDirs[ic] << " (camKey: " << camKey << ")" << endl;
			data.sources[camKey] = (ImageSource*) new ImageDirectory(data.imgDirs[ic]) ;
			
			data.camKey2Indx[camKey] = ic;
		}
	}
	else if( data.vidFiles.size() > 0 )
	{
		for( unsigned ic = 0; ic < data.vidFiles.size(); ++ic )
		{
			int a = data.vidFiles[ic].rfind( "/", data.vidFiles[ic].size()-2 )+1;
			int b = data.vidFiles[ic].find( ".", a );
			camKey = std::string( data.vidFiles[ic].begin() + a, data.vidFiles[ic].begin() + b );
			
			
			cout << "creating source: " << data.vidFiles[ic] << " with " << data.calibFiles[ic] << " (camKey: " << camKey << ")" << endl;
			data.sources[camKey] = (ImageSource*) new VideoSource(data.vidFiles[ic], data.calibFiles[ic]);
			
			data.camKey2Indx[camKey] = ic;
		}
	}
	else
	{
		cout << "no data.imgDirs nor data.vidFiles." << endl;
		exit(0);
	}
}
