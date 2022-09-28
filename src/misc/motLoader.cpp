#include "misc/motLoader.h"
#include "misc/tokeniser.h"
#include <fstream>

#include <iostream>
using std::cout;
using std::endl;

void ReadOpenSimMotFile( std::string fname, std::vector<std::string> &colNames, genMatrix &data, bool &isDegrees )
{
	// parse the header
	std::string cmd = "";
	bool doneHeader = false;
	
	int numRows, numCols;
	
	std::ifstream infi(fname);
	if( !infi )
		throw( std::runtime_error("Failed reading .mot file : could not open file: " + fname) );
	
	while(infi && !doneHeader )
	{
		infi >> cmd;
		if( !infi )
			continue;
		
		
		if( cmd.compare("endheader") == 0 )
		{
			doneHeader = true;
		}
		
		std::vector<std::string> ss;
		ss = SplitLine( cmd, " =\t" );
		
		// what is the line telling us?
		if( ss[0].compare("nRows") == 0)
		{
			numRows = atoi( ss[1].c_str() );
		}
		else if( ss[0].compare("nColumns") == 0)
		{
			numCols = atoi( ss[1].c_str() );
		}
		else if( ss[0].compare("inDegrees") == 0)
		{
			if( ss[1].compare("yes") == 0 )
				isDegrees = true;
			else
				isDegrees = false;
		}
	}
	
	if( !infi || !doneHeader)
		throw( std::runtime_error("Failed reading mot file during header") );
	
	// now we can read the data.
	data = genMatrix::Zero( numRows, numCols );
	colNames.resize( numCols );
	
	for( unsigned cc = 0; cc < numCols; ++cc )
	{
		infi >> colNames[cc];
	}
	
	if( !infi )
		throw( std::runtime_error("Failed reading mot file during column names") );
	
	for( unsigned rc = 0; rc < numRows; ++rc )
	{
		for( unsigned cc = 0; cc < numCols; ++cc )
		{
			infi >> data(rc,cc);
		}
	}
	
	if( !infi )
		throw( std::runtime_error("Failed reading mot file during data") );
	
	
}
