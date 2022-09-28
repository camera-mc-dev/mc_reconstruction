#include "misc/vtpLoader.h"
#include "misc/tokeniser.h"

#include <vector>
#include <string>



std::shared_ptr< Rendering::Mesh > LoadVTPFile( std::string filename )
{
	std::ifstream infi( filename );
	if( !infi )
		throw std::runtime_error("could not open .vtp file: " + filename );
	
	// we should use a nice xml library... meh.
	
	// read lines until we get PolyData
	bool gotPolyData = false;
	while( !gotPolyData && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("PolyData") == 0 )
		{
			gotPolyData = true;
		}
	}
	
	if( !gotPolyData )
	{
		throw std::runtime_error("Did not find PolyData in mesh file: " + filename );
	}
	
	// look for the <Points> section.
	bool gotPointSection = false;
	while( !gotPointSection && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("Points") == 0 )
		{
			gotPointSection = true;
		}
	}
	
	if( !gotPointSection )
	{
		throw std::runtime_error("Did not find Points in mesh file: " + filename );
	}
	
	
	// find the points DataArray...
	bool gotPointsDataArray = false;
	while( !gotPointsDataArray && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("DataArray") == 0 )
		{
			gotPointsDataArray = true;
		}
	}
	
	if( !gotPointsDataArray )
	{
		throw std::runtime_error("Did not find Polys DataArray in mesh file: " + filename );
	}
	
	// read the vertices.
	int c = 0;
	std::vector< hVec3D > points;
	gotPointsDataArray = false;
	hVec3D v;
	while( !gotPointsDataArray && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("/Points") == 0 )
		{
			gotPointSection = true;
			continue;
		}
		
		if( tokens.size() > 0 && tokens[0].find("/DataArray") != std::string::npos )
		{
			gotPointsDataArray = true;
			continue;
		}
		else
		{
			for( unsigned tc = 0; tc < tokens.size(); ++tc )
			{
				v(c) = atof( tokens[tc].c_str() );
				++c;
				if( c == 3 )
				{
					v(3) = 1.0f;
					c = 0;
					points.push_back(v);
				}
			}
		}
		
	}
	
	if( !gotPointSection )
	{
		throw std::runtime_error("Did not finish Points in mesh file: " + filename );
	}
	
// 	cout << "got : " << points.size() << " vertices " << endl;
	
	
	// find the faces (Polys) section
	bool gotPolySection = false;
	while( !gotPolySection && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("Polys") == 0 )
		{
			gotPolySection = true;
		}
	}
	
	if( !gotPolySection )
	{
		throw std::runtime_error("Did not find Polys in mesh file: " + filename );
	}
	
	// find the connectivity DataArray...
	bool gotPolyDataArray = false;
	while( !gotPolyDataArray && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("DataArray") == 0 )
		{
			for( unsigned tc = 0; tc < tokens.size(); ++tc )
			{
				if( tokens[tc].find("Name=\"connectivity\"") != std::string::npos )
				{
					gotPolyDataArray = true;
				}
			}
		}
	}
	
	if( !gotPolyDataArray )
	{
		throw std::runtime_error("Did not find Polys DataArray in mesh file: " + filename );
	}
	
	std::vector< int > conArray;
	gotPolyDataArray = false;
	while( !gotPolyDataArray )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("\DataArray") == 0 )
		{
			gotPolyDataArray = true;
		}
		else
		{
			for( unsigned tc = 0; tc < tokens.size(); ++tc )
			{
				conArray.push_back( atoi( tokens[tc].c_str() ) );
			}
		}
	}
	
	if( !gotPolyDataArray )
	{
		throw std::runtime_error("Did not read Polys connectivity DataArray in mesh file: " + filename );
	}
	
	
	gotPolyDataArray = false;
	while( !gotPolyDataArray && infi )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("DataArray") == 0 )
		{
			for( unsigned tc = 0; tc < tokens.size(); ++tc )
			{
				if( tokens[tc].find("Name=\"offsets\"") != std::string::npos )
				{
					gotPolyDataArray = true;
				}
			}
		}
	}
	
	if( !gotPolyDataArray )
	{
		throw std::runtime_error("Did not find Polys offset DataArray in mesh file: " + filename );
	}
	
	std::vector< int > offArray;
	gotPolyDataArray = false;
	while( !gotPolyDataArray )
	{
		std::string line;
		std::getline( infi, line );
		
		if( !infi )
			continue;
		
		auto tokens = SplitLine( line, "<> \t" );
		
		if( tokens.size() > 0 && tokens[0].compare("\DataArray") == 0 )
		{
			gotPolyDataArray = true;
		}
		else
		{
			for( unsigned tc = 0; tc < tokens.size(); ++tc )
			{
				offArray.push_back( atoi( tokens[tc].c_str() ) );
			}
		}
	}
	
	if( !gotPolyDataArray )
	{
		throw std::runtime_error("Did not read Polys offset DataArray in mesh file: " + filename );
	}
	
	
	// now we can finally build the faces
	std::vector< std::vector<int> > faces;
	int o0 = 0;
	for( unsigned oc = 0; oc < offArray.size(); ++oc )
	{
		int o1 = offArray[oc];
		
		assert( o1 - o0 == 3 ); // otherwise not a triangle and I can't handle it!
		
		std::vector<int> face(3);
		face[0] = conArray[ offArray[o0] ];
		face[1] = conArray[ offArray[o0+1] ];
		face[2] = conArray[ offArray[o0+2] ];
		faces.push_back(face);
	}
	

	
	
// 	cout << "got : " << faces.size() << " faces " << endl;
	
	// initialise mesh. Assuming triangle mesh. sucks to be you 
	// if that's not right.
	std::shared_ptr<Rendering::Mesh> mesh( new Rendering::Mesh( points.size(), faces.size()) );
	for( unsigned pc = 0; pc < points.size(); ++pc )
		mesh->vertices.col(pc) = points[pc];
	for( unsigned fc = 0; fc < faces.size(); ++fc )
	{
		mesh->faces.col(fc) << faces[fc][0], faces[fc][1], faces[fc][2];
	}
	
	mesh->CalculateMeanNormals(); // I mean, the file might have them but, meh.
	
	return mesh;
	
}
