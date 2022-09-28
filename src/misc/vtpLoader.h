#ifndef VTP_LOADER_H
#define VTP_LOADER_H

#include "renderer2/mesh.h"
#include <string>

// this is a _very_ simplistic function for loading .vtp mesh files.
// Not that it is by no means a robust and reliable file loader.
// it is, in truth, a very questionable hack.
// only handles .ascii files, and even then, only the ones I needed.
std::shared_ptr< Rendering::Mesh > LoadVTPFile( std::string filename );

#endif
