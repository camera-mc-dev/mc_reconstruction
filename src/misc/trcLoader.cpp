#include "trcLoader.h"
#include <iostream>
using std::cout;
using std::endl;

#include <fstream>

#include "misc/tokeniser.h"

void LoadTRCFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points )
{
	std::ifstream infi( filename );
	if( !infi )
	{
		throw( std::runtime_error( "Could not open file: " + filename ));
	}
	
	cout << "Note: .trc file loader is trash hacking right now. Use at your peril." << endl;
	
	// read all the lines
	std::vector< std::string > lines;
	while( infi )
	{
		std::string line;
		std::getline( infi, line );
		lines.push_back( line );
	}
	
	// what I _think_ is going on:
	//
	// line 0: stuff I don't care about.
	// line 1: names for things in line 2
	// line 2: some data as per line 1
	// line 3: headings for main table
	// line 4: sub-headings for main table
	// line 5 -> : data.
	
	auto metaDataNames = SplitLine(lines[1], "\t");
	auto metaData      = SplitLine(lines[2], "\t");
	auto mainNames     = SplitLine(lines[3], "\t");
	auto subNames      = SplitLine(lines[4], "\t");
	
	assert( metaDataNames.size() == metaData.size() );
	
	// expectation is that main names is: "Frame", "Time", "<marker name 0>, <marker name 1>, ...., <marker name n>" 
	std::vector< std::string > markerNames( mainNames.begin()+2, mainNames.end() );
	
	int c = 0;
	while( metaDataNames[c].compare("NumMarkers") != 0 && c < metaDataNames.size() )
		++c;
	assert( c < metaDataNames.size() );
	int numMarkers = atoi(metaData[c].c_str());
	
	cout << mainNames.size() << " " << markerNames.size() << " " << numMarkers << endl;
	assert( markerNames.size() == numMarkers );
	
	c = 0;
	while( metaDataNames[c].compare("NumFrames") != 0 && c < metaDataNames.size() )
		++c;
	assert( c < metaDataNames.size() );
	int numFrames = atoi(metaData[c].c_str());
	
	// TODO: Might need "firstFrame" in the future.
	cout << "trc loader: ignoring first frame 'cus I can't be bothered - so if it isn't 1, things may be off a bit in time..." << endl;
	startFrame = 0;
	
	assert( subNames.size() == markerNames.size() * 3 ); // I'm assuming x y z for each name
	
	for( unsigned mc = 0; mc < markerNames.size(); ++mc )
	{
		// to remain compatible with our .c3d loader
		// the matrix will store x,y,z,1.0, r, v
		// where r is the residual, and v is 1 for valid or 0 for not valid.
		// we don't have a residual here, so we'll set it to 0.
		points[ markerNames[mc] ] = genMatrix::Zero( 6, numFrames );
	}
	
	cout << lines.size() << endl;
	for( unsigned lc = 5; lc < lines.size(); ++lc )
	{
		auto dataStrings = SplitLine(lines[lc], "\t");
		if( dataStrings.size() != 2+ (markerNames.size() * 3) )
			continue;
			
		int frame = atoi( dataStrings[0].c_str() ) - 1; // why do people still index from 1? Honestly.
		int idx = 2;
		for( unsigned mc = 0; mc < markerNames.size(); ++mc )
		{
			points[ markerNames[mc] ].col( frame ) << 1000*atof( dataStrings[idx].c_str() ), 1000*atof(dataStrings[idx+1].c_str()), 1000*atof(dataStrings[idx+2].c_str() ), 1.0, 0.0, 1.0;
			idx+=3;
			
		}
	}

}
