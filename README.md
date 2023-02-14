README FOR LZROVERENGINE 
====================================

Made for the Lunar Zebro project, as a part of the Technical University of Delft, Netherlands

this 'engine' as I like to call it, is capable of:
 - emulating the sensor inputs of a rover on a lunar environment 
 - filtering this data for noise
 - clustering it into groundplane and separate clusters

The engines sensor inputs and visual runtime debug environment is based upon a GLFW / OpenGL pipeline + window. For now the window is a requirement, even if a debug viewer isn't needed.

The techniques used for:
 - the general navigation pipeline
 - dynamic voxel grid
 - normal estimation
 - clustering

are inspired, if not just in some cases copied from a series of closely related papers, of which the main ones are listed below:
 - 'SegMap: Segment-based mapping and localization using data-driven descriptors' DOI: 10.1177/0278364919863090
 - 'SegMatch: Segment Based Place Recognition in 3D Point Clouds' DOI: 10.1109/ICRA.2017.7989618
 - 'Incremental Segment-Based Localization in 3D Point Clouds'  DOI: ???

INSTALLATION
====================================

This software has only been tested on Ubuntu 20.04+ and Windows 10, but is likely to work on any linux system.
With some extra dependency linking efforts, it is also possible to get it running within Windows, which has been used through development to allow usage of specific profilers.

Dependencies:
 - gcc
 - cmake
 - OpenGL
 - GLUT
 - glfw3
 - glad
 - eigen
 - assimp

Installation guide for Ubuntu (should be similar for any linux system):
 - gcc

Enables compilation of the C++ source code

```$ sudo apt-get install build-essential ```
 - cmake

Project build tool that simplifies source file relations and dependencies and the like

???
 - OpenGL:

Gives a set of functions for graphics calculations

```$ sudo apt-get update```

```$ sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev```
 - GLUT:

Gives an extended set of functionality on top of OpenGL

This should already be installed through the OpenGL installation step
 - glfw3:

Gives an extended set of functionality on top of OpenGL focused on windows and user interaction

```$ sudo apt-get install libglfw3 libglfw3-dev``` 
 - glad

OpenGL thingy

Go to https://glad.dav1d.de/, choose your openGL version, disable all options except 'Local Files' and click 'Generate'. Copy the 'khr_platform' and 'glad.c' file into your root/usr/local/include/glad folder. If this doesn't exist, make this folder new
 - eigen

Go to https://eigen.tuxfamily.org/index.php?title=Main_Page, and download the 3.4.0 stable release. Extract it, and copy the resulting folder to your root/usr/local/ folder. rename the folder to 'eigen3'.
 - assimp

A set of functions enabling the loading of a large set of 3D-model file formats.

```$ sudo apt-get install libassimp-dev```

RUNNING THE PROGRAM
====================================

After git cloning the repository, cd to LZRoverEngine/, and make a new build directory

```$ git clone git@github.com/IvoBlok/LZRoverEngine.git```

```$ cd LZRoverEngine/```

```$ mkdir build```

Then, let cmake setup the make files given the source directory

```$ cd build/```

```$ cmake ../```

To build the project, run the following

```$ cmake --build .```

To execute the project, run the compiled executable

```$ ./LZRoverEngine```

If after taking these steps, a change is made to the source code, only the last two steps are needed to run the updated source code.

DEBUG CONTROLS
====================================
For movement of the debug camera, use WASD to move in the plane defined by the debug camera's up direction as it's normal vector, and spacebar and left shift to move up and down the previously mentioned vector respectively.
Some other controls can be modified from the settings.h file and/or might need to be enabled with another setting in the same file.

DEBUGGING and/or PROFILING
====================================
For any future (and current) contributors to the codebase, this 'chapter' is intended to rougly document how to debug the program.
So far the development of this project has been on windows 10 using WSL and VcXsrv to run it on Ubuntu 20.04. For debugging all things memory, the code is debugged and/or profiled using the built-in debugger of Visual Studio Code on a dedicated Ubuntu 20.04 machine by cloning the repo.

Mainly for profiling and finding general issues, A windows version of the project can be made by means that are still to be documented (I have no clue how I managed to get it working), that can be built and run using Visual Studio. For profiling, I so far have been using Optick, see here: https://github.com/bombomby/optick. This requires you to make minor changes to the codebase to signal to Optick what you specifically want to profile, giving you more control of what you want to see and compare. To profile the executable using Optick, run the executable and start the optick GUI. If the modifications to the codebase we're done correct, Optick will start collecting profiling stuff. By starting the optick GUI as administrator, it can give you even finer detail (and seems to ignore all specifications to what functions you want to profile, and just include everything) on what's been happening.

QUESTIONS / HELP
====================================

As a horrible way of futureproofing, if you happened to be working with or on this project and you have a serious problem, feel free to mail me:
 - ivoblokdoorn@gmail.com
