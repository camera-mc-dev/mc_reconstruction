#ifndef MC_DEV_TRC_LOADER_H
#define MC_DEV_TRC_LOADER_H

#include <map>
#include "math/mathTypes.h"

void LoadTRCFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points );

#endif
