==========
README.txt
==========
Readme to the SSE Particle Filter Codebase,
the SSE Wrapper Library, and the Observation
Generator.


=======
LICENSE
=======
This software is released under the MIT license.
Please see LICENSE.txt for more details.


===============
Version History
===============
Version 0.1 - Initial release.  October 9, 2009.


=============
Prerequisites
=============
Linux
1) g++.  Tested with g++ ver. 4.3.3.
2) OpenGL and glut must be installed.
   On Ubuntu, get the "freeglut3-dev" package.

Windows
1) Microsoft Visual Studio 2005 or 2008
2) Intel C++ Compiler.  Tested with version 10.0.


===================================================
Notes on using SSE Particle Filter Codebase (ssePf)
===================================================

Compilation
-----------
Linux - Run "make" from the top level directory.  You
        may need to add the path to OpenGL and glut.

Windows - Open msvs_icc/particle_filter.sln.

          Be sure the "Solution Configuration"
          dropdown is set to "Release."  By default
          it is set to "Debug."

          You will need to have the Intel C++ Compiler
          installed because the codebase is not
          compatible with the Visual Studio compiler.

          Select Build -> Build Solution.

Running
-------
Run "particle_filter" in the top level directory.
The file "sim_obs.csv" is needed by the particle filter
and should be located in this directory.


GUI controls
------------
'~' - toggles between SSE and scalar mode 
tab - changes the display filter (4 versions)
left/right - move to previous/next observation
up/down - increase/decrease the observation window
'q' - quit

Compile-time options
--------------------
The following can be changed at compile time in
"main.cpp".

1) OBS_FILENAME can be changed to load a different
file at startup.

2) Enable the first #if block to test for the particle
filter's accuracy in SSE mode compared to scalar mode.

3) Enable the second #if block to compare the accuracy
of the SSE math functions with respect to the reference
versions in math.h.


========================================
Notes on using SSE Wrapper Library (sse)
========================================
This wrapper library consists of a set of C++ header files.
It does not compile into a library.  To use the library,
include either of the two header files at the top of any
source file that needs its functionality:

  1) sse/sse.h - basic SSE wrapper types
  2) sse/sseMath.h - includes everything in sse/sse.h and
                     also SSE versions of math.h functions


=============================================
Notes on using Observation Generator (obsGen)
=============================================
A simple Python script for generating the observation
file used by the SSE particle filter.  Requires Python.


=====
Outro
=====
I hope you find this software is useful!

 - Peter D.

// end of README.txt
