import os,sys

sys.path.append("../mc_core/")
import mcdev_core_config as corecfg

# ---------------------------------------------------------
# results of some checks.
# ---------------------------------------------------------
openmpAvailable = True

# ---------------------------------------------------------
# Set libraries to link against and include paths for the compiler
# that are common across all build modes
# ---------------------------------------------------------
def SetPathsLibsAndFlagsAll(env):
	
	corecfg.SetCoreConfig(env)
	env.Append(CPPPATH=["%s/../mc_core/src"%(Dir('.').abspath)])
	env.Append(CPPPATH=["%s/../mc_sds/src"%(Dir('.').abspath)])
	



# ---------------------------------------------------------
# Set libraries to link against and include paths for the compiler
# specific to the debug build
# ---------------------------------------------------------
def SetPathsLibsAndFlags_debug(env):
	env.Append(CPPFLAGS=['-g', '-O0'])	#-ggdb or -glldb may be better.





# ---------------------------------------------------------
# Set libraries to link against and include paths for the compiler
# specific to dbgOpt build
# ---------------------------------------------------------
def SetPathsLibsAndFlags_dbgOpt(env):
	# debugging will always work better with -O0
	# but sometimes you need to have debug symbols in the optimised
	# stuff.
	env.Append(CPPFLAGS=['-g', '-O2'])
	env.Append(CPPFLAGS=['-fopenmp'])




# ---------------------------------------------------------
# Set libraries to link against and include paths for the compiler
# specific to optimised build
# ---------------------------------------------------------
def SetPathsLibsAndFlags_opt(env):
	env.Append(CPPFLAGS=['-O2'])
	env.Append(CPPFLAGS=['-fopenmp'])


# ---------------------------------------------------------
# Set libraries to link against and include paths for the compiler
# specific to the dbg-memSan build. Clang supplies a memorySanitizer
# tool that might, perhaps, make up for the fact valgrind doesn't
# fully work on OSX.
# ---------------------------------------------------------
def SetPathsLibsAndFlags_memSan(env):
	env.Append(CPPFLAGS=['-g', '-O1', '-fsanitize=address', '-fno-omit-frame-pointer'])
	env.Append(LINKFLAGS=['-g', '-O1', '-fsanitize=address', '-fno-omit-frame-pointer'])




















# ---------------------------------------------------------
# Determine the build mode    : debug, dbgOpt, optimised
#                    debug    : debugging symbols enabled, no optimisations
#                    dbgOpt   : debugging symbols enabled, optimisations enabled
#                    optimised: no debug symbols, full optimisations and openmp support
# ---------------------------------------------------------

buildMode = ARGUMENTS.get('mode','optimised')
showCommand = ARGUMENTS.get('showCommand', 'false')

# make sure the build mode is valid
if buildMode not in ['debug', 'dbg-memSan', 'dbgOpt', 'optimised']:
	print( "Build mode can only be 'debug', 'dbgOpt', 'dbg-memSan' or 'optimised', but you asked for: ", buildMode )
	exit(1)

print( "***  Compiling in %s mode..."%buildMode )


# define a few colours for output prettiness.
# these are bash "escape codes" that set the text colours of
# compatible terminals. We'll use these colours in just a
# a moment....
colors = {}
colors['cyan']   = '\033[96m'
colors['purple'] = '\033[95m'
colors['blue']   = '\033[94m'
colors['green']  = '\033[92m'
colors['yellow'] = '\033[93m'
colors['red']    = '\033[91m'
colors['end']    = '\033[0m'

# If the output is not a terminal, remove the colors we just defined :)
# (yes, I may have just copy-pasted these colouring sections to save me time...
if not sys.stdout.isatty():
	for key, value in colors.iteritems():
		colors[key] = ''

# -------------------------------------------------------
# Create the "Master" build environment. This is the
# build settings that are common between all of our build modes.
#
# We do this in one of two ways:
# 1) in the default case, where showCommand is false, we hide the actual
#    build command in favour of a simple short message. It's just a little cleaner.
# 2) in the second case, where showCommand is true, we do SCons' default behaviour
#    of displaying the build command. i.e. if we're compiling a file, we'll see
#    the full g++ fred.cpp -o geoff -L ....
# -------------------------------------------------------
masterEnv = None
if showCommand == 'false':
	compile_source_message = '%sCompiling %s==> %s$SOURCE%s' % \
	(colors['blue'], colors['purple'], colors['yellow'], colors['end'])

	compile_shared_source_message = '%sCompiling shared %s==> %s$SOURCE%s' % \
	(colors['blue'], colors['purple'], colors['yellow'], colors['end'])

	link_program_message = '%sLinking Program %s==> %s$TARGET%s' % \
	(colors['cyan'], colors['purple'], colors['yellow'], colors['end'])

	link_library_message = '%sLinking Static Library %s==> %s$TARGET%s' % \
	(colors['cyan'], colors['purple'], colors['yellow'], colors['end'])

	ranlib_library_message = '%sRanlib Library %s==> %s$TARGET%s' % \
	(colors['cyan'], colors['purple'], colors['yellow'], colors['end'])

	link_shared_library_message = '%sLinking Shared Library %s==> %s$TARGET%s' % \
	(colors['cyan'], colors['purple'], colors['yellow'], colors['end'])

	java_library_message = '%sCreating Java Archive %s==> %s$TARGET%s' % \
	(colors['cyan'], colors['purple'], colors['yellow'], colors['end'])

	masterEnv = Environment(
	CXXCOMSTR = compile_source_message,
	CCCOMSTR = compile_source_message,
	SHCCCOMSTR = compile_shared_source_message,
	SHCXXCOMSTR = compile_shared_source_message,
	ARCOMSTR = link_library_message,
	RANLIBCOMSTR = ranlib_library_message,
	SHLINKCOMSTR = link_shared_library_message,
	LINKCOMSTR = link_program_message,
	JARCOMSTR = java_library_message,
	JAVACCOMSTR = compile_source_message
	)
elif showCommand == 'true':
	masterEnv = Environment()
else:
	print( "showCommand must be set to 'true' or 'false'" )
	print( "default is 'false', which minimises the output when building." )
	print( "if set to 'true', build will display each build command." )
	exit(1)

#
# Find out the version of gcc on the system from the environment
# and if it is modern enough (>4.9) enable coloured compiler output.
#
try:
    masterEnv['ENV']['TERM'] = os.environ['TERM']
    if masterEnv['CCVERSION'] > "4.9":
        masterEnv.Append(CPPFLAGS=['-fdiagnostics-color=auto'])
except:
    pass


# and now do libraries, cflags etc that are common to all build modes
SetPathsLibsAndFlagsAll(masterEnv)

# ---------------------------------------------------------------
# Having created our master build environment, we now clone
# it to create our specific build environments for optimised, debug,
# and debug-optimised compilation.
#  - debug:  no optimisations, full debug symbols
#  - dbgOpt: optimisations enabled, full debug symbols
#  - optimised: optimisations enabled, no debug symbols (default)
#
#----------------------------------------------------------------
buildEnvs = {}
buildEnvs['debug'] = masterEnv.Clone()
buildEnvs['dbgOpt'] = masterEnv.Clone()
buildEnvs['dbg-memSan'] = masterEnv.Clone()
buildEnvs['optimised'] = masterEnv.Clone()

SetPathsLibsAndFlags_debug( buildEnvs['debug'])
SetPathsLibsAndFlags_dbgOpt( buildEnvs['dbgOpt'])
SetPathsLibsAndFlags_memSan( buildEnvs['dbg-memSan'])
SetPathsLibsAndFlags_opt( buildEnvs['optimised'])


# --------------------------------------------------------------
# Now that we have configured our build environment, we want to
# actually tell SCons what we are going to build.
#
# We build a library using all the source under src/
# including sub-directories.
#
# then we have various directories for different types of
# programs that will link against the library.
# 1) apps:        These are programs that implement the solution to a specific
#                 research problem. They will be grouped in project subdirectory.
# 2) utils:       These are small single purpose utilities not affiliated to
#                 a specific research project.
# 3) experiments: These are incomplete, temprorary, or opportunistic daliances
#                 to try something quickly and roughly.
# 4) tests:       These are small programs to tests specific functionality.
#                 I shan't say unit tests, but, well... unit tests.
#
#
# These are child SConscript files to handle each of the above.
#
# --------------------------------------------------------------

# for each build mode, set up the build directory,
# and make use of each sub SConscript
for bm in buildEnvs:
	buildDir = "build/%s"%bm
	env = buildEnvs[bm]

	env.SConscript(dirs=['src'], variant_dir="%s/src"%buildDir, exports ='env buildDir bm', duplicate=False)
	env.SConscript(dirs=['apps'], variant_dir="%s/apps"%buildDir, exports ='env buildDir bm', duplicate=False)
	env.SConscript(dirs=['tests'], variant_dir="%s/tests"%buildDir, exports ='env buildDir bm', duplicate=False)
	


# ------------------------------------------------------------
# One last thing we do is tell SCons what to build - otherwise it will just build all
# of the build modes. This is done by calling the 'Default' function, however,
# if we're cleaning, we probably do want to clean _all_ of the build modes, and not just
# the specified build mode. In that case, we don't set the default and just leave it.
# ------------------------------------------------------------
if not GetOption("clean"):
	Default('build/%s'%buildMode)
