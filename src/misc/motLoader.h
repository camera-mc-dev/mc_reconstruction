#ifndef MOT_LOADER_H
#define MOT_LOADER_H


#include "math/mathTypes.h"

//
// loader for open sim mot file.
//
void ReadOpenSimMotFile( std::string fname, std::vector<std::string> &colNames, genMatrix &data, bool &isDegrees );


#endif
