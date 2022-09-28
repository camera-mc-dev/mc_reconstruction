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
	
	//
	// And iterate over the image frames
	//
	do
	{
		int fc = isrc->GetCurrentFrameID();
		img = isrc->GetCurrent();
		
		auto pi = poseData.find(fc);
		for( unsigned pc = 0; pc < pi->second.size(); ++pc )
		{
			PersonPose &person = pi->second[pc];
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
	}
	while( isrc->Advance() );
}
