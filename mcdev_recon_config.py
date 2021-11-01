from SCons.Script import *

def FindC3D(env):
	# we can use ezc3d for loading .c3d files.
	if os.path.isdir("/usr/local/include/ezc3d") and os.path.isdir("/usr/local/lib/ezc3d"):
		env.Append(LIBPATH=["/usr/local/lib/ezc3d"])
		env.Append(CPPPATH=["/usr/local/include/ezc3d"])
		env.Append(LIBS=["ezc3d"])
		env.Append(CPPDEFINES=["USE_EZC3D"])

def SetReconConfig(env):
	FindC3D(env)
