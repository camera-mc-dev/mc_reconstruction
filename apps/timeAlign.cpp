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
	
	float lowMedian, highMedian;
	
	bool visualise;
	bool usePrevious;
	int  maxBlinkSignalLag;
	std::string brightDataFilename;
	
};

void ParseConfig( std::string configFile, SData &data );
void GetSources( SData &data );


void FilterFlashOn(genMatrix brightData, genMatrix &out )
{
	Eigen::VectorXf filter(30);
	for( unsigned cc = 0; cc < filter.rows(); ++cc )
	{
		if( cc < filter.rows()/2 ) filter(cc) = -1.0f;
		else filter(cc) = 1.0f;
	}
	out = genMatrix::Zero( brightData.rows(), brightData.cols() );
	for( unsigned cc = 0; cc < brightData.cols() - filter.rows(); ++cc )
	{
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			Eigen::VectorXf w = brightData.block(rc,cc, 1, filter.rows()).transpose();
			out(rc,cc+filter.rows()/2) = (w.array() * filter.array()).sum() / (float)filter.rows();  // cc+filter.rows()/2 so that signal peaks on the event
		}
	}
	
}
void FilterFlashOff(genMatrix brightData, genMatrix &out )
{
	Eigen::VectorXf filter(30);
	for( unsigned cc = 0; cc < filter.rows(); ++cc )
	{
		if( cc < filter.rows()/2 ) filter(cc) = 1.0f;
		else filter(cc) = -1.0f;
	}
	out = genMatrix::Zero( brightData.rows(), brightData.cols() );
	for( unsigned cc = 0; cc < brightData.cols() - filter.rows(); ++cc )
	{
		for( unsigned rc = 0; rc < brightData.rows(); ++rc )
		{
			Eigen::VectorXf w = brightData.block(rc,cc, 1, filter.rows()).transpose();
			out(rc,cc+filter.rows()/2) = (w.array() * filter.array()).sum() / (float)filter.rows();  // cc+filter.rows()/2 so that signal peaks on the event
		}
	}
}

void CrossViewFilter( genMatrix inData, genMatrix &out )
{
	Eigen::VectorXf gfilter(60);
	out = inData;
	
	Eigen::VectorXf gscv;
	float numRpt = 5.0f;
	for(unsigned rpt = 0; rpt < numRpt; ++rpt )
	{
		// create the gaussian filter.
		for( unsigned cc = 0; cc < 60; ++cc )
		{
			float x = cc - 30.0;
			float sig = 5.0f; //(11-rpt);
			//gfilter(cc) = exp( -(x*x)/(sig*sig)  );
			gfilter(cc) = exp( -pow( abs(x), 0.8 ) / 0.3 );
			cout << std::setw(6) << std::fixed << std::setprecision(2) << gfilter(cc);
		}
		float gsc = gfilter.array().sum();
		
		// Apply the gaussian filter to get our 1D response
		gscv = Eigen::VectorXf::Zero(inData.cols());
		for( unsigned cc = 0; cc < inData.cols() - 60; ++cc )
		{
			for( unsigned rc = 0; rc < inData.rows(); ++rc )
			{
				Eigen::VectorXf w = out.block(rc,cc, 1, 60).transpose();
				gscv(cc+30) += (w.array() * gfilter.array()).sum()/gsc;
			}
			gscv(cc+30) /= inData.rows();
		}
		
		
		//
		// Then push each viewpoint towards that filtered mean
		//
		for( unsigned rc = 0; rc < inData.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < inData.cols(); ++cc )
			{
				// too lazy to do this properly.
				//out(rc,cc) = (numRpt-1.0)/numRpt * out(rc,cc)  + 1.0/numRpt * gscv(cc);
				out(rc,cc) = 0.5 * out(rc,cc)  + 0.5 * gscv(cc);
			}
		}
	}
}

void PeakDetect( genMatrix inData, genMatrix &out, bool offToOn )
{
	out = genMatrix::Zero( inData.rows(), inData.cols() );
	float thr = (inData.mean() + inData.maxCoeff()) / 2.0f;
	cout << "inData m, mn, M: " << inData.minCoeff() << " " << inData.mean() << " " << inData.maxCoeff() << endl;
	std::ofstream plog( "peak.log" );
	for( unsigned rc = 0; rc < inData.rows(); ++rc )
	{
		int peakStart = -1;
		for( unsigned cc = 0; cc < inData.cols(); ++cc )
		{
			if(
			    inData(rc,cc) > thr             && 
			    inData(rc,cc) > inData(rc,cc-3) &&
			    inData(rc,cc) > inData(rc,cc+3)
			  )
			{
				out(rc,cc) = 1.0f;
				if( peakStart < 0 )
					peakStart = cc;
			}
			else if( peakStart >= 0 )
			{
				int peakEnd = cc;
				if( peakEnd - peakStart > 1 )
				{
					plog << "wide peak : " << peakStart << " -> " << peakEnd << endl << "\t";
					if( offToOn )
						plog << "offToOn" << endl;
					else
						plog << "onToOff" << endl;
					float peakLoc = 0.0f;
					float wsum    = 0.0f;
					for( unsigned cc2 = peakStart; cc2 < peakEnd; ++cc2 )
					{
						//
						// The weight of each part of the peak is probably equal, so
						// the mean we calculate here always ends up in the middle. Which is fine.
						// _But_ it almost always seems to be an onToOff, and it feels like it is
						// always pulling thing too late, so we're trying to pull it earlier with
						// an augmentation of the weight and by using the floor on the average.
						//
						float w = inData(rc,cc);
						float aug = 0.0f;
						if( !offToOn )
						{
							aug = (peakEnd-1.0f-cc2) / (float)(peakEnd-peakStart);
						}
						else
						{
							aug = (cc2-peakStart) / (float)(peakEnd-peakStart);
						}
						plog << cc2 << "(" << w << ", " << aug << ") ";
						peakLoc += (w+aug) * cc2;
						wsum    += (w+aug);
					}
					plog << endl;
					
					peakLoc /= wsum;
					plog << "\t" << peakLoc << " -> ";
					peakLoc = floor(peakLoc);
					plog << peakLoc << endl;
					
					
					for( unsigned cc = peakStart; cc < peakEnd; ++cc )
					{
						if(cc != peakLoc)
							out(cc) = 0;
					}
				}
				peakStart = -1;
			}
		}
	}
	
	
}


void PeakCheck( genMatrix inData, genMatrix &out )
{
	out = genMatrix::Zero( inData.rows(), inData.cols() );
	for( unsigned cc = 0; cc < inData.cols(); ++cc )
	{
		int c = 0;
		for( unsigned rc = 0; rc < inData.rows(); ++rc )
		{
			if( inData(rc,cc) == 1.0f )
				++c;
		}
		if( c > inData.rows() / 2 )
		{
			for( unsigned rc = 0; rc < inData.rows(); ++rc )
			{
				out(rc,cc) = 1.0f;
			}
		}
	}
}


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
		cout << argv[0] << " <config file> | <use previous (0 = false,1=true)|" << endl;
		exit(0);
	}
	
	SData data;
	ParseConfig( argv[1], data );
	GetSources( data );
	
	if( argc == 3 )
	{
		data.usePrevious = atoi( argv[2] ) == 1;
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
		else
		{
			cout << "we do." << endl;
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
	std::shared_ptr<Rendering::MeshNode> tstNode;
	std::vector< std::vector<float> > meanBrights;
	genMatrix brightData;
	if( !data.usePrevious )
	{
		cout << "playing through video data to learn light pattern..." << endl;
		cout << "takes a wee-while normally" << endl;
		cv::Mat tst(4, data.sources.begin()->second->GetNumImages(), CV_32FC3, cv::Scalar(0) );
		if( data.visualise )
		{
			tstNode  = Rendering::GenerateImageNode(0,   0, img.cols, 50, tst, "tstNode" , ren);
			ren->Get2dFgRoot()->AddChild( tstNode );
			tstNode->GetTexture()->UploadImage(    tst      );
		}
		while(!done && fc0 < tst.cols )
		{
			if( fc0 % 30 == 0 )
				cout << fc0 << " / " << data.sources.begin()->second->GetNumImages() << endl;
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
						
						if( sidi == data.sources.begin() )
						{
							tst.at<cv::Vec3f>(0, fc0)[0] += p[0];
							tst.at<cv::Vec3f>(1, fc0)[1] += p[1];
							tst.at<cv::Vec3f>(2, fc0)[2] += p[2];
							
							tst.at<cv::Vec3f>(3, fc0)[0] += bright;
							tst.at<cv::Vec3f>(3, fc0)[1] += bright;
							tst.at<cv::Vec3f>(3, fc0)[2] += bright;
							tst.at<cv::Vec3f>(3, fc0) /= 3.0f;
						}
					}
				}
				bright /= 32*32;
				
				brights[ id2ind[ sidi->first ] ] = bright;
			}
			meanBrights.push_back( brights );
			
			if( data.visualise )
			{
				float n = 255 * (32*32);
				tst.at<cv::Vec3f>(0, fc0)[0] /= n;
				tst.at<cv::Vec3f>(1, fc0)[1] /= n;
				tst.at<cv::Vec3f>(2, fc0)[2] /= n;
				
				tst.at<cv::Vec3f>(3, fc0)[0] /=n;
				tst.at<cv::Vec3f>(3, fc0)[1] /=n;
				tst.at<cv::Vec3f>(3, fc0)[2] /=n;
				tstNode->GetTexture()->UploadImage(    tst      );
			}
			
			
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
				//cv::circle( img, cv::Point( bcp(0), bcp(1) ), 15, cv::Scalar(255,255,0), 2 );
				cv::rectangle( img, cv::Rect( bcp(0)-16, bcp(1)-16, 32, 32 ), cv::Scalar(255,255,0), 2 );
				
				
				
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
			float md = rowVals[ 0.25*rowVals.size() ]; //rowVals[ rowVals.size()/2 ];
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
	genMatrix bd2On, bd2Off;
	FilterFlashOn( brightData, bd2On );
	FilterFlashOff( brightData, bd2Off );
	
	bd = cv::Mat( brightData.rows()*10, brightData.cols(), CV_32FC1, cv::Scalar(0.0f) );
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 5; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = bd2On(rc,cc);
			}
			for( unsigned rc2 = 5; rc2 < 10; ++rc2 )
			{
				float &p = bd.at<float>( rc*10 + rc2, cc );
				p = bd2Off(rc,cc);
			}
		}
	}
	if( data.visualise )
		Rendering::ShowImage(bd, 1500);
	
	
	//
	// If we gaussian filter across time using all the views, we can create cross-view consistency (?)
	// 
	genMatrix bd2OnG, bd2OffG;
	CrossViewFilter( bd2On, bd2OnG );
	CrossViewFilter( bd2Off, bd2OffG );
	
	
	
	if( data.visualise )
	{
		for( unsigned rc = 0; rc < bd2On.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd2On.cols(); ++cc )
			{
				for( unsigned rc2 = 0; rc2 < 5; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = bd2OnG(rc,cc);
				}
				for( unsigned rc2 = 5; rc2 < 10; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = bd2OffG(rc,cc);
				}
			}
		}
		Rendering::ShowImage(bd, 1500);
	}
	
	
	
	
	
	
	
	
	//
	// Now we find the brightness peaks. They should be strong enough that we can make use of a threshold
	// as well as a local peak filter.
	//
	genMatrix peakOn, peakOff;
	PeakDetect(  bd2OnG,  peakOn,  true );
	PeakDetect( bd2OffG, peakOff, false );
	
	
	
	if( data.visualise )
	{
		for( unsigned rc = 0; rc < bd2On.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd2On.cols(); ++cc )
			{
				for( unsigned rc2 = 0; rc2 < 5; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = peakOn(rc,cc);
				}
				for( unsigned rc2 = 5; rc2 < 10; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = peakOff(rc,cc);
				}
			}
		}
		Rendering::ShowImage(bd, 1500);
	}
	
	
	//
	// Now we use the multiple cameras to get robustness with a simple count.
	//
	genMatrix peakOn2, peakOff2;
	PeakCheck(  peakOn, peakOn2 );
	PeakCheck( peakOff, peakOff2 );
	
	
	
	if( data.visualise )
	{
		for( unsigned rc = 0; rc < bd2On.rows(); ++rc )
		{
			for( unsigned cc = 0; cc < bd2On.cols(); ++cc )
			{
				for( unsigned rc2 = 0; rc2 < 5; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = peakOn2(rc,cc);
				}
				for( unsigned rc2 = 5; rc2 < 10; ++rc2 )
				{
					float &p = bd.at<float>( rc*10 + rc2, cc );
					p = peakOff2(rc,cc);
				}
			}
		}
		Rendering::ShowImage(bd, 1500);
	}
	
	
	//
	// I _really_ hate the ugly mess that I've used previously here, even though I know I've done a load 
	// of work to make it robust - but it is kind of impenetrable and hard to update.
	//
	// So here's a new version! Take that previous Murray.
	//
	
	
	//
	// Put those peaks into a simple signal.
	//
	genMatrix tmp = peakOn2 + -1.0 * peakOff2;
	Eigen::VectorXf flashSignal( tmp.cols() );
	for( unsigned c = 0; c < tmp.cols(); ++c )
	{
		flashSignal(c) = tmp.col(c).mean();
	}
	if( data.visualise )
	{
		for( unsigned rc = 0; rc < bd.rows; ++rc )
		{
			for( unsigned cc = 0; cc < bd.cols; ++cc )
			{
				float &p = bd.at<float>( rc, cc );
				p = 0.5 + 0.5 * flashSignal(cc);
			}
		}
		Rendering::ShowImage(bd, 1500);
	}
	
	//
	// Now make the same kind of signal with "blinky"
	//
	Eigen::VectorXf blinkSignal;
	if( data.useChannel < 0 )
	{
		blinkSignal = Eigen::VectorXf::Zero( data.tracks[ledName].cols() );
		for( unsigned cc = 0; cc < data.tracks[ledName].cols()-1; ++cc )
		{
			Eigen::VectorXf c0 = data.tracks[ledName].col(cc);
			Eigen::VectorXf c1 = data.tracks[ledName].col(cc+1);
			
			if( c0(0) == 0.0f && c0(1) == 0.0f && c0(2) == 0.0f &&
			  ( c1(0) != 0.0f && c1(1) != 0.0f && c1(2) != 0.0f)  )
			{
				blinkSignal(cc) = 1.0f;
			}
			else if ( c0(0) != 0.0f && c0(1) != 0.0f && c0(2) != 0.0f &&
			        ( c1(0) == 0.0f && c1(1) == 0.0f && c1(2) == 0.0f)  )
			{
				blinkSignal(cc) = -1.0f;
			}
		}
	}
	else
	{
		// the analog signal is faster than the video signal, so need a scale for frame numbers.
		float sc = data.channels.cols() / minElements;
		
		// this is the raw signal.
		// I _think_ the channel goes -ve for on, near 0 for off.
		Eigen::VectorXf blinkData = data.channels.row( data.useChannel );
		
		// what is the mean value of the channel?
		float m  = blinkData.minCoeff();
		float mn = blinkData.mean();
		float M  = blinkData.maxCoeff();
		cout << "blinkData m, mn, M: " << m << " " << mn << " " << M << endl;
		
		// init the blinkSignal.
		blinkSignal = Eigen::VectorXf::Zero( blinkData.rows() / sc );
		cout << blinkSignal.rows() << endl;
		// fill the blink signal.
		for( unsigned cc = 0; cc < data.channels.cols()-1; ++cc )
		{
			float v0 = data.channels( data.useChannel, cc   );
			float v1 = data.channels( data.useChannel, cc+1 );
			
			if( v0 > mn && v1 < mn )
			{
				blinkSignal(cc/sc) =  1.0f;
				cout << " on: " << cc << " ( " << cc/sc << " ) " << endl;
			}
			if( v0 < mn && v1 > mn )
			{
				blinkSignal(cc/sc) = -1.0f;
				cout << " off: " << cc << " ( " << cc/sc << " ) " << endl;
			}
		}
	}
	
	cv::Mat bd2( 100, blinkSignal.rows(), CV_32FC1, cv::Scalar(0.0f) );
	if( data.visualise )
	{
		for( unsigned rc = 0; rc < bd2.rows; ++rc )
		{
			for( unsigned cc = 0; cc < bd2.cols; ++cc )
			{
				float &p = bd2.at<float>( rc, cc );
				p = 0.5 + 0.5 * blinkSignal(cc);
			}
		}
		Rendering::ShowImage(bd2, 1500);
	}
	
	
	// offset is:
	// mocapFrame = vidFrame + offset
	//
	// Make a 2 row matrix which is as long as if you concat the blink signal at the front and end of the 
	// flash signal.
	//
	// Make each element of the first row state its minimum distance to a flash start,
	// and each element of the second row state its minimum distance to a flash end.
	//
	genMatrix bsp = genMatrix::Constant( 2, blinkSignal.rows() + flashSignal.rows() + blinkSignal.rows(), -1.0f );
	std::vector<int> s,e;
	for( unsigned c = 0; c < flashSignal.rows(); ++c )
	{
		if( flashSignal(c) > 0 )
			s.push_back( c+blinkSignal.rows() );
		else if( flashSignal(c) < 0 )
			e.push_back( c+blinkSignal.rows() );
	}
	
	for( unsigned c = blinkSignal.rows(); c < blinkSignal.rows()  + flashSignal.rows(); ++c )
	{
		float d = bsp.cols();
		for( unsigned sc = 0; sc < s.size(); ++sc )
		{
			d = std::min( d, std::abs( (float)c - s[sc] ) );
		}
		bsp(0,c) = d;
		
		d = bsp.cols();
		for( unsigned ec = 0; ec < e.size(); ++ec )
		{
			d = std::min( d, std::abs( (float)c - e[ec] ) );
		}
		bsp(1,c) = d;
	}
	
	
	//
	// The last problem that we have is if we were stupid enough to have some kind
	// of repeating pattern to the flash, making it non-unique across the sequence.
	// Which we were for BioCV.
	//
	// In that case there could be multiple best fits - so find them all.
	//
	// To find the best fits, we're going to slide the blink signal over the flash signal
	// which is to say, the markers over the image signal.
	//
	// We "know" that the blink signal wont end _that_ long after the flash signal - typically
	// just a few frames. So we prevent going too far beyond the end of the flash signal.
	//
	// We can specify how much the end of the blink signal allowed to lag the end of the flash signal
	// in the config option: maxBlinkSignalLag
	
	std::vector<float> offErrs;
	for( unsigned offset0 = blinkSignal.rows() / 2; offset0 < flashSignal.rows() + data.maxBlinkSignalLag; ++offset0 )
	{
		float d = 0.0f;
		int cnt = 0;
		for( unsigned c = 0; c < blinkSignal.rows(); ++c )
		{
			float s = bsp(0, c + offset0);
			float e = bsp(1, c + offset0);
			if( blinkSignal(c) > 0 && (s>=0 || e>=0) )
			{
				d += bsp(0, c + offset0);
				++cnt;
			}
			else if( blinkSignal(c) < 0 && (s>=0 || e>=0))
			{
				d += bsp(1, c + offset0);
				++cnt;
			}
		}
		if( cnt > 2 )
		{
			d /= cnt;
			offErrs.push_back(d);
		}
		else
		{
			offErrs.push_back(9999);
		}
		cout << offset0 << "( " << (int)offset0 - blinkSignal.rows()/2 << " ) : "  << cnt <<  " : " << d << " " << offErrs.back() << endl;
	}
	
	std::vector<float> offErrsCopy = offErrs;
	std::sort( offErrsCopy.begin(), offErrsCopy.end() );
	float medErr = offErrsCopy[ offErrsCopy.size() * 0.3 ];
	float thrErr = (medErr + offErrsCopy[0]) / 2.0f;
	int minOffset0;
	{
		std::vector<int> offMins;
		int s, m, e;
		s = e = m = -1;
		float minErr = 9999.99f;
		for( unsigned ec = 0; ec < offErrs.size(); ++ec )
		{
			if( offErrs[ec] < thrErr )
			{
				s = ec;
				if( s < 0 )
					minErr = offErrs[ec];
				else
				{
					if( offErrs[ec] < minErr )
					{
						minErr = offErrs[ec];
						m = ec;
					}
				}
			}
			else if( s >= 0 )
			{
				offMins.push_back( m );
				s = -1;
				m = -1;
				minErr = 9999.99f;
			}
			
			cout << ec << " ( " << ec + blinkSignal.rows()/2 << " ) : " << medErr << " ? " << offErrs[ec] << " : " << (offErrs[ec] < medErr) << " <> " << m << endl;
		}
		
		cout << "got " << offMins.size() << " possible offsets: " << endl;
		for( unsigned oc = 0; oc < offMins.size(); ++oc )
		{
			cout << offMins[oc] + blinkSignal.rows() / 2 << " : " << offErrs[ offMins[oc] ] << endl;
		}
		cout << "taking the last one in case of repeating flash pattern." << endl;
		minOffset0 = offMins.back() + blinkSignal.rows() / 2;
	}
	
	
	// which means that our best offset is at...
	// NOTE: The +1 is a bit arbitrary. When we _look_ at the alignImg it seems like the solve 
	//       puts us one frame late. If we put this +1 in, the blink and flash _start_ at the same time,
	//       even though the flash can often linger a frame longer. Some of this all comes down to just 
	//       exactly how Qualsys processed the flashing led... perhaps...
	int finalOffset = -(minOffset0 - blinkSignal.rows()) + 1;

	cout << "final offset: " << finalOffset << endl;
	
	int numVidFrames = brightData.cols();
	int numMocapFrames = data.tracks.begin()->second.cols();
	cout << numVidFrames << " " << numMocapFrames << endl;
	int baseOffset = numMocapFrames - numVidFrames;
	
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
	
	
	//
	// I want to render the bright data with an overlay of the mocap led data, at the final alignment.
	//
	cv::Mat alignImg( brightData.rows()*10, brightData.cols(), CV_32FC3, cv::Scalar(0.0f) );
	cout << "bd cv shape: " << bd.rows << " " << bd.cols << endl;
	for( unsigned rc = 0; rc < brightData.rows(); ++rc )
	{
		for( unsigned cc = 0; cc < brightData.cols(); ++cc )
		{
			for( unsigned rc2 = 0; rc2 < 10; ++rc2 )
			{
				cv::Vec3f &p = alignImg.at<cv::Vec3f>( rc*10 + rc2, cc );
				p[0] = brightData(rc,cc);
				
				if( cc + finalOffset > 0 && cc + finalOffset < data.tracks[ledName].cols() )
					p[1] = data.tracks[ledName].col( cc + finalOffset ).head(3).norm();
				
			}
		}
	}
	std::stringstream aiss;
	aiss << data.dataRoot << "/" << data.testRoot << "/talign.png";
	SaveImage( alignImg, aiss.str() );
	
	
	if( data.visualise )
	{
		if( data.visualise )
			Rendering::ShowImage(alignImg, 1500);
		
		
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
				
// 				cv::line( img, cv::Point( a(0), a(1) ), cv::Point( b(0), b(1) ), cv::Scalar(128,  0,   0), 2);
// 				cv::line( img, cv::Point( b(0), b(1) ), cv::Point( c(0), c(1) ), cv::Scalar(255,  0,   0), 2);
// 				cv::line( img, cv::Point( c(0), c(1) ), cv::Point( d(0), d(1) ), cv::Scalar(  0,  0, 255), 2);
// 				cv::line( img, cv::Point( d(0), d(1) ), cv::Point( e(0), e(1) ), cv::Scalar(  0,  0, 128), 2);
			}
			
			
			//
			// Draw all markers
			//
			for( auto ti = data.tracks.begin(); ti != data.tracks.end(); ++ti )
			{
				if( fc-2 + finalOffset < 0 || fc+2 + finalOffset >=  ti->second.cols() )
					continue;
				
				if( ti->first.compare(ledName) == 0 )
					continue;
				
				hVec2D bcp = calib.Project( ti->second.col( fc + finalOffset ).head(4) );
				cv::circle( img, cv::Point( bcp(0), bcp(1) ), 5, cv::Scalar(0,255,0), 2 );
				
			}
			
			
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
		
		if( cfg.exists("maxBlinkSignalLag") )
		{
			data.maxBlinkSignalLag = cfg.lookup("maxBlinkSignalLag");
		}
		else
			data.maxBlinkSignalLag = 50;
		
		
		data.lowMedian = 0.25;
		if( cfg.exists("lowMedian") )
		{
			data.lowMedian = cfg.lookup("lowMedian");
		}
		data.highMedian = 0.75;
		if( cfg.exists("highMedian") )
		{
			data.highMedian = cfg.lookup("highMedian");
		}
		
		assert( data.lowMedian >= 0 && data.lowMedian < data.highMedian );
		assert( data.highMedian > 0 && data.highMedian > data.lowMedian && data.highMedian <= 1.0f );
		
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
