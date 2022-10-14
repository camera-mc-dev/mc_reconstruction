#include "recon/poseFusion/poseFusion.h"
#include "imgio/sourceFactory.h"
#include "commonConfig/commonConfig.h"

#include "renderer2/basicRenderer.h"

int main( int argc, char *argv[] )
{
	if( argc != 4 )
	{
		cout << "Tool to render sparse pose detections" << endl;
		cout << "Usage: " << endl;
		cout << argv[0] << "<image source> <pose type> <pose source>" << endl;
		exit(0);
	}
	
	//
	// get the image source.
	//
	auto isrc = CreateSource( argv[1] ).source;
	
	
	//
	// what type of pose data is it?
	//
	std::string pstr( argv[2] );
	skeleton_t skelType;
	if( pstr.compare("open") == 0 || pstr.compare("openpose") == 0 )
	{
		skelType = SKEL_OPOSE;
	}
	if( pstr.compare("alpha") == 0 || pstr.compare("alphapose") == 0 )
	{
		skelType = SKEL_APOSE;
	}
	if( pstr.compare("dlc") == 0 || pstr.compare("deeplabcut") == 0 )
	{
		skelType = SKEL_DLCUT;
	}
	
	//
	// Load the pose data
	//
	std::map< int, std::vector<PersonPose> > poseData;
	switch( skelType )
	{
		case SKEL_OPOSE:
		case SKEL_APOSE:
			ReadPoseDirJSON( skelType, argv[3], poseData );
			break;
		
		case SKEL_DLCUT:
			ReadDLC_CSV( pstr, poseData );
			break;
	}
	
	
	//
	// Make the renderer
	//
	cv::Mat img = isrc->GetCurrent();
	
	// how big are the images?
	float ar = img.rows/ (float) img.cols;
	
	
	// create the renderer
	std::shared_ptr< Rendering::BasicRenderer > ren;
	
	cout << "creating window" << endl;
	CommonConfig ccfg;
	float winW = ccfg.maxSingleWindowWidth;
	float winH = winW * ar;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	Rendering::RendererFactory::Create( ren, winW, winH, "sparse pose vis");
	
	ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
	ren->Get2dFgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10);
	
	// we want to draw lines between some subsets of the keypoints
	std::vector< std::pair<int,int> > leftLines, rightLines, midLines;
	switch( skelType )
	{
		case SKEL_OPOSE:
			rightLines.push_back( std::pair<int,int>(1,2) );
			rightLines.push_back( std::pair<int,int>(2,3) );
			rightLines.push_back( std::pair<int,int>(3,4) );
			
			rightLines.push_back( std::pair<int,int>(8,9)  );
			rightLines.push_back( std::pair<int,int>(9,10) );
			rightLines.push_back( std::pair<int,int>(10,11));
			
			midLines.push_back( std::pair<int,int>(1,8)  );
			
			leftLines.push_back( std::pair<int,int>(1,5) );
			leftLines.push_back( std::pair<int,int>(5,6) );
			leftLines.push_back( std::pair<int,int>(6,7) );
			
			leftLines.push_back( std::pair<int,int>(8,12) );
			leftLines.push_back( std::pair<int,int>(12,13));
			leftLines.push_back( std::pair<int,int>(13,14));
			
			break;
		case SKEL_APOSE:
			rightLines.push_back( std::pair<int,int>(1,2) );
			rightLines.push_back( std::pair<int,int>(2,3) );
			rightLines.push_back( std::pair<int,int>(3,4) );
			
			rightLines.push_back( std::pair<int,int>(1,8) );
			rightLines.push_back( std::pair<int,int>(8,9) );
			rightLines.push_back( std::pair<int,int>(9,10));
			
			
			
			leftLines.push_back( std::pair<int,int>(1,5) );
			leftLines.push_back( std::pair<int,int>(5,6) );
			leftLines.push_back( std::pair<int,int>(6,7) );
			
			leftLines.push_back( std::pair<int,int>(1, 11));
			leftLines.push_back( std::pair<int,int>(11,12));
			leftLines.push_back( std::pair<int,int>(12,13));
			
			break;
		
		case SKEL_DLCUT:
			// Not bothering.
			break;
	}
	
	//
	// And iterate over the image frames
	//
	cv::Scalar leftColour( 255, 0, 0 );
	cv::Scalar rightColour( 0, 255, 0);
	cv::Scalar midColour( 255, 255, 0);
	do
	{
		int fc = isrc->GetCurrentFrameID();
		img = isrc->GetCurrent();
		
		auto pi = poseData.find(fc);
		for( unsigned pc = 0; pc < pi->second.size(); ++pc )
		{
			PersonPose &person = pi->second[pc];
			
			for( unsigned llc = 0; llc < leftLines.size(); ++llc )
			{
				hVec2D p0 = person.joints[ leftLines[llc].first  ];
				hVec2D p1 = person.joints[ leftLines[llc].second ];
				
				float c0 = person.confidences[leftLines[llc].first];
				float c1 = person.confidences[leftLines[llc].second];
				
				if( c0 > 0.3 && c1 > 0.3 )
					cv::line( img, cv::Point(p0(0), p0(1)), cv::Point(p1(0),p1(1)), leftColour, 2 );
			}
			for( unsigned mlc = 0; mlc < midLines.size(); ++mlc )
			{
				hVec2D p0 = person.joints[ midLines[mlc].first  ];
				hVec2D p1 = person.joints[ midLines[mlc].second ];
				
				float c0 = person.confidences[leftLines[mlc].first];
				float c1 = person.confidences[leftLines[mlc].second];
				
				if( c0 > 0.3 && c1 > 0.3 )
					cv::line( img, cv::Point(p0(0), p0(1)), cv::Point(p1(0),p1(1)), midColour, 2 );
			}
			for( unsigned rlc = 0; rlc < rightLines.size(); ++rlc )
			{
				hVec2D p0 = person.joints[ rightLines[rlc].first  ];
				hVec2D p1 = person.joints[ rightLines[rlc].second ];
				
				float c0 = person.confidences[leftLines[rlc].first];
				float c1 = person.confidences[leftLines[rlc].second];
				
				if( c0 > 0.3 && c1 > 0.3 )
					cv::line( img, cv::Point(p0(0), p0(1)), cv::Point(p1(0),p1(1)), rightColour, 2 );
			}
			
			for( unsigned jc = 0; jc < person.joints.size(); ++jc )
			{
				hVec2D p = person.joints[jc];
				float  c = person.confidences[jc];
				
				int b,g,r;
				b = 256 * c;
				g = 0;
				r = 256 * c;
				cv::circle( img, cv::Point( p(0), p(1) ), 4, cv::Scalar( b,g,r ), 2 );
			}
		}
		ren->SetBGImage(img);
		
		ren->StepEventLoop();
// 		auto g = ren->Capture();
// 		std::stringstream ss;
// 		ss << "spRender/" << std::setw(6) << std::setfill('0') << fc << ".jpg";
// 		SaveImage( g, ss.str() );
	}
	while( isrc->Advance() );
}
