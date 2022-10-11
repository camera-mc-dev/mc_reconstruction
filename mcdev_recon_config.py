from SCons.Script import *

def FindC3D(env):
	# we can use ezc3d for loading .c3d files.
	if os.path.isdir("/usr/local/include/ezc3d") and os.path.exists("/usr/local/lib/libezc3d.so"):
		env.Append(LIBPATH=["/usr/local/lib/ezc3d/"])
		env.Append(CPPPATH=["/usr/local/include/ezc3d"])
		env.Append(LIBS=["ezc3d"])
		env.Append(CPPDEFINES=["USE_EZC3D"])
	else:
		print( "Could not find ezc3d, check paths in mcdev_recon_config.py, FindC3D()" )

def FindOpensim(env):
	env.Append(CPPDEFINES=["USE_OPENSIM"]);
	env.Append(CPPPATH=["/home/muzz/programming/notmine/opensim/opensim_install/include/",
	                    "/home/muzz/programming/notmine/opensim/opensim_install/include/OpenSim/",
	                    "/home/muzz/programming/notmine/opensim/opensim_install/include/simbody/" ])
	env.Append(LIBPATH=["/home/muzz/programming/notmine/opensim/opensim_install/lib/"])
	env.Append(LIBS=["osimAnalyses", "osimActuators", "osimSimulation", "osimTools", "osimCommon", "SimTKsimbody", "SimTKcommon"])

def FindAssImp(env):
	env.ParseConfig("pkg-config assimp --cflags --libs");

def SetReconConfig(env):
	FindC3D(env)
	FindOpensim(env)
	FindAssImp(env)
