#ifndef C3D_LOADER_H

#ifdef USE_EZC3D

#include <map>
#include "math/mathTypes.h"

void LoadC3DFile( std::string filename, unsigned &startFrame, std::map< std::string, genMatrix > &points );



#endif


#endif
