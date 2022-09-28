#include <iostream>
using std::cout;
using std::endl;
#include <fstream>
#include <sstream>


#ifdef USE_OPENSIM

#include <OpenSim/OpenSim.h>
namespace osim = OpenSim;

#include "math/mathTypes.h"
#include "renderer2/basicRenderer.h"
#include "renderer2/basicHeadlessRenderer.h"
#include "misc/vtpLoader.h"
#include "misc/motLoader.h"

#include "imgio/imagesource.h"
#include "imgio/vidsrc.h"
#include "commonConfig/commonConfig.h"
#include "math/matrixGenerators.h"


#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// #define USE_HEADLESS

// spit out 4x4 transformations to disk
// sparse fusion
// dense fusion
// fit smpl 2 dp, silhouettes


class OpenSimConverter
{
public:
	OpenSimConverter( std::string modelFile, std::string motionFile, std::string geometryDir );
	
	
	// Initialise render nodes (must provide the renderer)
	// returns the root node.
#ifdef USE_HEADLESS
	void InitialiseRenderNodes( std::shared_ptr<Rendering::BasicHeadlessRenderer> &ren );
#else
	void InitialiseRenderNodes( std::shared_ptr<Rendering::BasicPauseRenderer> &ren );
#endif
	
	// get access to the root node.
	std::shared_ptr< Rendering::SceneNode > RootNode() {return rootNode;}
	
	
	
	
	// set pose as per frame number
	void SetPose( unsigned frameNo );
	
	void DumpPose(std::string poseFile);
	
protected:
	
	struct SMeshData
	{
		// the scene graph should do this for us, but this makes it easy to find things.
		std::string meshFile;
		std::shared_ptr<Rendering::Mesh> mesh;
		std::shared_ptr<Rendering::MeshNode> node;
		transMatrix3D gS;
	};
	
	// mesh stuff for each body part.
	std::map< std::string, std::vector< SMeshData > > meshData;
	
	// we'll have a scene node for each body part.
	std::map< std::string, std::shared_ptr<Rendering::SceneNode> > partNodes;
	
	// root Node!
	std::shared_ptr<Rendering::SceneNode> rootNode;
	
	
	
	// The OpenSim model
	osim::Model model;
	SimTK::State modelState;
	
	// The motion data. I hope.
	std::vector<std::string> colNames;
	std::map<std::string, unsigned> varName2Col;
	genMatrix motData;
	bool isDegrees;
	
};

// global but... fuck it.
int syncOffset;

int main(int argc, char* argv[] )
{
	if( argc != 3 && argc != 7 && argc != 8 )
	{
		cout << "tool to test loading/rendering an OpenSim model"     << endl;
		cout << "on top of a calibrated image source."                << endl;
		cout <<                                                          endl;
		cout << "specify model file (.osim), motion file (.mot) and " << endl;
		cout << "directory containing relevant model geometry mesh  " << endl;
		cout << "files (.vtp probably)"                               << endl;
		cout << "calib file only need be supplied for video sources " << endl;
		cout << endl;
		cout << argv[0] << "<model file> <motion file> <geometry dir> <save dir> <syncFile> <image source> | <calib file>" << endl;
		cout << endl;
		cout << "or for bioCV data with assumed dataRoot of /data2/bioCV" << endl;
		cout << "<testRoot> <camera>" << endl;
		exit(0);
	}
	
	std::string modelFile, motionFile, geomDir, saveDir, source, calibFile, syncFile, poseDumpFile;
	bool providedCalibFile = false;
	
	if( argc == 3 )
	{
		std::string dataRoot = "/data2/bioCV/";
		std::string testRoot = argv[1];
		
// 		std::stringstream ss;
// 		ss << dataRoot << testRoot << "../biocv_fullbody_markerless_scaled.osim";
// 		modelFile = ss.str();
// 		
// 		ss.str("");
// 		ss << dataRoot << testRoot << "/op-fused/joints_kalman.mot";
// 		motionFile = ss.str();
		
		
		std::stringstream ss;
		ss << dataRoot << testRoot << "../biocv_fullbody_markers_scaled.osim";
		modelFile = ss.str();
		
		ss.str("");
		ss << dataRoot << testRoot << "/markers.mot";
		motionFile = ss.str();
		
		geomDir = "/data2/opensim/opensimGeometry/";
		
		ss.str("");
		ss << dataRoot << testRoot << "/openSimRender-" << argv[2] << "/";
		saveDir = ss.str();
		
		ss.str("");
		ss << dataRoot << testRoot << "/" << argv[2] << ".mp4";
		source = ss.str();
		
		ss.str("");
		ss << dataRoot << testRoot << "../" << argv[2] << ".mp4-mocAligned.calib";
		calibFile = ss.str();
		providedCalibFile = true;
		
		ss.str("");
		ss << dataRoot << testRoot << "/frameOffset";
		syncFile = ss.str();
	}
	else
	{
		modelFile  = argv[1];
		motionFile = argv[2];
		geomDir    = argv[3];
		
		saveDir = argv[4];
		syncFile = argv[5];
		
		source = argv[6];
		
		if( argc == 8 )
		{
			providedCalibFile = true;
			calibFile = argv[7];
		}
	}
	
	poseDumpFile = saveDir + "../openSimTransformations";
	
	std::ifstream syncfi( syncFile );
	if( !syncfi )
		throw std::runtime_error("Can't load sync file: " + syncFile );
	std::string x;
	syncfi >> x;
	assert( x.compare("offset:") == 0 );
	syncfi >> syncOffset;
	
	cout << "syncOffset: " << syncOffset << endl;
	sleep(3);
	
	// create the model wrapper/converter
	OpenSimConverter osimConverter(modelFile, motionFile, geomDir);
	osimConverter.DumpPose( poseDumpFile );
	cout << "temporarily skipping all rendering" << endl;
	
	// where we output to.
	if( boost::filesystem::is_directory( saveDir ))
	{
		cout << "Caution! Output dir already exists! Overwriting in... (Ctrl-c to abort)" << endl;
		for( unsigned c = 0; c < 5; ++c )
		{
			cout << 5-c << " ... " << endl;
			sleep(1);
		}
	}
	else
	{
		boost::filesystem::path p(saveDir);
		boost::filesystem::create_directories(p);
	}
	
	// initialise image source
	ImageSource* isrc;
	if( boost::filesystem::is_directory( source ))
	{
		isrc = (ImageSource*) new ImageDirectory( source );
	}
	else
	{
		// assume calib file is named like the video file.
		std::stringstream calibfn;
		if( !providedCalibFile )
		{
			calibfn << source << ".calib";
		}
		else
		{
			calibfn << calibFile;
		}
		if( boost::filesystem::exists( calibfn.str() ) )
		{
			isrc = (ImageSource*) new VideoSource( source, calibfn.str() );
		}
		else
		{
			cout << "Didn't find a calib file where expected: " << calibfn.str() << endl;
			exit(0);
		}
	}
	
	// initialise renderer & renderer calibs/projections.
	CommonConfig ccfg;
	cv::Mat img = isrc->GetCurrent();
	float ar = img.rows / (float)img.cols;
	float winW = ccfg.maxSingleWindowWidth;
	float winH = ar * winW;
	if( winH > ccfg.maxSingleWindowHeight )
	{
		winH = ccfg.maxSingleWindowHeight;
		winW = winH / ar;
	}
	
	
	cout << "win W, H: " << winW << ", " << winH << endl;
	
#ifdef USE_HEADLESS
	std::shared_ptr<Rendering::BasicHeadlessRenderer> ren;
#else
	std::shared_ptr<Rendering::BasicPauseRenderer> ren;
#endif
	Rendering::RendererFactory::Create( ren, winW,winH, "opensim renderer" );
	ren->Get2dBgCamera()->SetOrthoProjection(0, img.cols, 0, img.rows, -10, 10 );
	ren->Get3dCamera()->SetFromCalibration( isrc->GetCalibration(), 0.2, 20000 );
	
	ren->SetActive();
	glCullFace(GL_FRONT);
	
	
	// initialise the render nodes
	osimConverter.InitialiseRenderNodes(ren);
	
	// put the model's root node on our render tree.
	ren->Get3dRoot()->AddChild( osimConverter.RootNode() );
	
	
	// iterate over video frames...
	bool done = false;
	bool paused = true;
	bool advance = false;
	cv::Mat undist;
	while( !done )
	{
		// get current image.
		img = isrc->GetCurrent();
		undist = isrc->GetCalibration().Undistort(img);
		ren->SetBGImage(undist);
		
		// figure out the right frame number(?)
		unsigned fno = isrc->GetCurrentFrameID();
		
		// ask for the renderables to update for the current frame
		osimConverter.SetPose( fno );
		
		// do the render.
#ifdef USE_HEADLESS
		ren->StepEventLoop();
#else
		ren->Step(paused, advance);
		while( paused && !advance )
		{
			ren->Step(paused, advance);
		}
		advance = false;
#endif
		
		// capture and save (if wanted)
		cv::Mat grab = ren->Capture();
		std::stringstream ss;
		ss << saveDir << "/" << std::setw(6) << std::setfill('0') << fno << ".jpg";
		SaveImage( grab, ss.str() );
		
		// advance source, see if we're done
		done = !isrc->Advance();
	}
}


OpenSimConverter::OpenSimConverter( std::string modelFile, std::string motionFile, std::string geometryDir ) : 
  model( modelFile )
{
	//
	// Load the motion file data
	//
	ReadOpenSimMotFile( motionFile, colNames, motData, isDegrees );
	
	cout << "read motion data: " << motData.rows() << " x " << motData.cols() << endl;
	
	
	//
	// Init the model
	//
	modelState = model.initSystem();
	
	
	//
	// Map state variable names to columns of our .mot file data.
	//
	auto stateVarNames = model.getStateVariableNames();
	for( unsigned vnc = 0; vnc < stateVarNames.size(); ++vnc )
	{
		cout << stateVarNames[vnc] << endl;
		for( unsigned cnc = 0; cnc < colNames.size(); ++cnc )
		{
			if( stateVarNames[vnc].find("/value") != std::string::npos && stateVarNames[vnc].find(colNames[cnc]) != std::string::npos )
			{
				varName2Col[ stateVarNames[vnc] ] = cnc;
				cout << stateVarNames[vnc] << " -> " << cnc << " ( " << colNames[cnc] << " ) " << endl;
			}
		}
	}
	cout << varName2Col.size() << " " << colNames.size() << endl;
	assert( varName2Col.size() == colNames.size()-1 );
	
	//
	// we need to look at the model to find all the geometry files - i.e. renderable meshes for body parts.
	// Now, awkwardly, a body part might have more than one mesh associated with it.
	//
	//
	std::vector< std::string > geomFiles;
	
	auto body = model.getBodySet();
	size_t numParts = body.getSize();
	for( unsigned pc = 0; pc < numParts; ++pc )
	{
		osim::Body part = body[ pc ]; 
		cout << part.getName() << endl; // how joyous that I don't see this method in the API docs...
		
		// how do I find the geometry?
		// it takes bit of digging but we got there in the end.
		auto &g = part.getProperty_attached_geometry();
		for( unsigned gc = 0; gc < g.size(); ++gc )
		{
			const osim::Mesh &osMesh = static_cast< const osim::Mesh& > (g[gc]);
			
			SMeshData md;
			md.meshFile = osMesh.get_mesh_file();
			
			auto &sf = osMesh.getProperty_scale_factors();
			assert( sf.size() == 1 );
			md.gS = transMatrix3D::Identity();
			md.gS(0,0) = sf[0][0];
			md.gS(1,1) = sf[0][1];
			md.gS(2,2) = sf[0][2];
			
			cout << "\t" << md.meshFile << endl;
			cout << md.gS << endl;
			
			meshData[ part.getName() ].push_back( md );
		}
		
	}
	
	
	
	//
	// Load up all the meshes
	//
	
	// we'll need to scale the vertices as mm instead of m
	transMatrix3D S;
	S << 1000,    0,    0,   0,
	        0, 1000,    0,   0,
	        0,    0, 1000,   0,
	        0,    0,    0,   1;
	
	for( auto mdi = meshData.begin(); mdi != meshData.end(); ++mdi )
	{
		// mesh loading...
		cout << mdi->first << " " << mdi->second.size() << " : ";
		for( unsigned mc = 0; mc < mdi->second.size(); ++mc )
		{
			// load mesh file
			std::stringstream ss;
			//ss << geometryDir << "/" << mdi->second[mc].meshFile;
			//mdi->second[mc].mesh = LoadVTPFile(ss.str());
			
			ss << geometryDir << "/" << mdi->second[mc].meshFile << ".ply";
			Assimp::Importer assimp;
			const aiScene *scene = assimp.ReadFile( ss.str(), aiProcess_Triangulate );
			
			assert( scene->HasMeshes() );
			assert( scene->mNumMeshes == 1 );
			
			std::shared_ptr<Rendering::Mesh> m;
			m.reset( new Rendering::Mesh( scene->mMeshes[0]->mNumVertices, scene->mMeshes[0]->mNumFaces ) );
			
			for( unsigned vc = 0; vc < scene->mMeshes[0]->mNumVertices; ++vc )
			{
				m->vertices.col(vc) << scene->mMeshes[0]->mVertices[vc][0], scene->mMeshes[0]->mVertices[vc][1], scene->mMeshes[0]->mVertices[vc][2], 1.0f;
			}
			
			for( unsigned fc = 0; fc < scene->mMeshes[0]->mNumFaces; ++fc )
			{
				m->faces.col(fc) << scene->mMeshes[0]->mFaces[fc].mIndices[0], scene->mMeshes[0]->mFaces[fc].mIndices[1], scene->mMeshes[0]->mFaces[fc].mIndices[2];
			}
			
			//m->vertices = S * mdi->second[mc].gS * m->vertices;
			m->vertices = S * mdi->second[mc].gS * m->vertices;
			m->CalculateMeanNormals();
			mdi->second[mc].mesh = m;
			
			
			cout << mdi->second[mc].meshFile << "( " << mdi->second[mc].mesh->vertices.cols() <<  ", " << mdi->second[mc].mesh->faces.cols() << " ) " << endl;
			cout << mdi->second[mc].gS << endl;
			
			
			
			
// 			cout << " " << mdi->second[mc].meshFile << "( " << mdi->second[mc].mesh->vertices.cols() <<  ", " << mdi->second[mc].mesh->faces.cols() << " ) : ";
			
		}
		cout << endl;
		
	}
	
	
	//
	// Good to go, I hope!
	//
	
}

#ifdef USE_HEADLESS
void OpenSimConverter::InitialiseRenderNodes( std::shared_ptr<Rendering::BasicHeadlessRenderer> &ren )
#else
void OpenSimConverter::InitialiseRenderNodes( std::shared_ptr<Rendering::BasicPauseRenderer> &ren )
#endif
{
	// model will be grey/white
	Eigen::Vector4f grey; grey << 0.97, 0.97, 0.97, 1.0f;
	
	// initialise our root node.
	// OpenSim has an annoying y-up coordinate system and _insists_ on data being transformed to that 
	// before it gets used. As such, we applied a -90 degree rotation around x before we started, so
	// get rid of that here.
	Rendering::NodeFactory::Create(rootNode, "osimModelRootNode");
	transMatrix3D Rx = RotMatrix( 1, 0, 0, 3.14/2.0 );
	transMatrix3D R  = Rx;
	rootNode->SetTransformation(R);
	
	cout << "Creating nodes: " << endl;
	for( auto mdi = meshData.begin(); mdi != meshData.end(); ++mdi )
	{
		// initialise a scene node for this body part.
		Rendering::NodeFactory::Create( partNodes[ mdi->first ], mdi->first);
		
		// because we can get the "ground" transformation from the model for each 
		// body part, we can parent each body part to the root node.
		rootNode->AddChild( partNodes[ mdi->first ] );
		cout << "\t" << mdi->first << " : ";
		for( unsigned mc = 0; mc < mdi->second.size(); ++mc )
		{
			cout << mdi->second[mc].meshFile << " ";
			// create mesh node for this mesh.
			// mesh filename is probably good and unique for the mesh node name,
			// and if it is not unique we'll find out soon enough!
			std::shared_ptr< Rendering::MeshNode > n;
			Rendering::NodeFactory::Create(n, mdi->second[mc].meshFile);
			
			mdi->second[mc].mesh->UploadToRenderer(ren);
			
			n->SetMesh( mdi->second[mc].mesh );
			n->SetBaseColour( grey );
			n->SetTexture( ren->GetBlankTexture() );
			n->SetShader( ren->GetShaderProg("basicLitColourShader")  );
			
			mdi->second[mc].node = n;
			
			partNodes[ mdi->first ]->AddChild( n );
		}
		
		cout << endl;
	}
}

void OpenSimConverter::DumpPose(std::string poseFile)
{
	std::ofstream outfi( poseFile );
	if( !outfi )
	{
		cout << "Could not open pose dump file: " << poseFile << endl;
		return;
	}
	
	outfi << "# <data row> <vid frame>" << endl;
	outfi << "# \t<part name> <T44 as row0 row1 row2 row3> " << endl;
	outfi << "# NOTE: translation dimensions left as metres" << endl;
	
	for( unsigned rc = 0; rc < motData.rows(); ++rc )
	{
		outfi << rc << " " << rc + syncOffset << endl;
		
		auto stateVarNames = model.getStateVariableNames();
		for( unsigned vnc = 0; vnc < stateVarNames.size(); ++vnc )
		{
			auto vni = varName2Col.find( stateVarNames[vnc] );
			if( vni == varName2Col.end() )
			{
				// this is a variable the file doesn't supply. Set to zero.
				cout << stateVarNames[vnc] << " := " << 0 << endl;
				model.setStateVariableValue( modelState, stateVarNames[vnc], 0 );
			}
			else if(vni->first.find("_tx") != std::string::npos || vni->first.find("_ty") != std::string::npos || vni->first.find("_tz") != std::string::npos)
			{
				// this is a translation/position variable.
				cout << vni->first << " := " << motData(rc, vni->second ) << endl;
				model.setStateVariableValue( modelState, vni->first, motData(rc, vni->second) );
			}
			else // if( vni->first.find("pelvis_list") != std::string::npos )
			{
				// this is an angle. The file supplies everything in degrees and yet we seem to have to input as radians?
				cout << vni->first << " := " << motData(rc, vni->second ) << " ( " << motData(rc, vni->second )*3.1415/180.0f << " ) " << endl;
				model.setStateVariableValue( modelState, vni->first, motData(rc, vni->second )*3.1415/180.0f );
			}
		}
		model.realizePosition( modelState );
		
		auto cl = model.getComponentList< osim::Frame >();
		for( auto cli = cl.begin(); cli != cl.end(); ++cli )
		{
			std::string name = cli->getName();
			
			auto Tsim = cli->getTransformInGround(modelState).toMat44();
			transMatrix3D Teig = transMatrix3D::Identity();
			for( unsigned rc = 0; rc < 4; ++rc )
			{
				for( unsigned cc = 0; cc < 4; ++cc )
				{
					Teig(rc,cc) = Tsim(rc,cc);
				}
			}
			
			outfi << "\t" << name << Teig.row(0) <<"\t" << Teig.row(1) << "\t" << Teig.row(2) << "\t" << Teig.row(3) << endl;
		}
		outfi << endl;
	}
}


void OpenSimConverter::SetPose( unsigned frameNo )
{
	if( (int)frameNo + (int)syncOffset < 0 )
		return;
	
	auto stateVarNames = model.getStateVariableNames();
	for( unsigned vnc = 0; vnc < stateVarNames.size(); ++vnc )
	{
		auto vni = varName2Col.find( stateVarNames[vnc] );
		if( vni == varName2Col.end() )
		{
			// this is a variable the file doesn't supply. Set to zero.
			cout << stateVarNames[vnc] << " := " << 0 << endl;
			model.setStateVariableValue( modelState, stateVarNames[vnc], 0 );
		}
		else if(vni->first.find("_tx") != std::string::npos || vni->first.find("_ty") != std::string::npos || vni->first.find("_tz") != std::string::npos)
		{
			// this is a translation/position variable.
			cout << vni->first << " := " << motData(frameNo + syncOffset, vni->second ) << endl;
			model.setStateVariableValue( modelState, vni->first, motData(frameNo + syncOffset, vni->second) );
		}
		else // if( vni->first.find("pelvis_list") != std::string::npos )
		{
			// this is an angle. The file supplies everything in degrees and yet we seem to have to input as radians?
			cout << vni->first << " := " << motData(frameNo + syncOffset, vni->second ) << " ( " << motData(frameNo + syncOffset, vni->second )*3.1415/180.0f << " ) " << endl;
			model.setStateVariableValue( modelState, vni->first, motData(frameNo + syncOffset, vni->second )*3.1415/180.0f );
		}
	}
	model.realizePosition( modelState );
	
	auto cl = model.getComponentList< osim::Frame >();
	for( auto cli = cl.begin(); cli != cl.end(); ++cli )
	{
		std::string name = cli->getName();
		
		auto ni = partNodes.find( name );
		if( ni != partNodes.end() )
		{
			auto Tsim = cli->getTransformInGround(modelState).toMat44();
			transMatrix3D Teig = transMatrix3D::Identity();
			for( unsigned rc = 0; rc < 4; ++rc )
			{
				for( unsigned cc = 0; cc < 4; ++cc )
				{
					Teig(rc,cc) = Tsim(rc,cc);
					if( cc == 3 && rc < 3)
						Teig(rc,cc) *= 1000.0f;
				}
			}
			
			if( name.compare("pelvis") == 0 )
			{
				cout << frameNo << endl;
				cout << name << endl;
				cout << Tsim << endl;
				cout << Teig << endl << endl;
				
// 				cout << RotMatrix(1,0,0, frameNo*3.1415/180.0f) << endl;
			}
			
			
			ni->second->SetTransformation(Teig);
		}
	}
}

#else

int main(int argc, char* argv[] )
{
	cout << "Framework not compiled with OpenSim enabled (USE_OPENSIM != True)" << endl;
	return 0;
}

#endif
