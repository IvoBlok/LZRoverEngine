====================================
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

these techniques are inspired, and if not just copied from a series of closely related papers, of which the main ones are listed below:
 - 'SegMap: Segment-based mapping and localization using data-driven descriptors' DOI: 10.1177/0278364919863090
 - 'SegMatch: Segment Based Place Recognition in 3D Point Clouds' DOI: 10.1109/ICRA.2017.7989618
 - 'Incremental Segment-Based Localization in 3D Point Clouds' 

INSTALLATION

This software has only been tested on Ubuntu 20.04+, but is likely to work on any linux system, and with some extra dependency linking efforts, it should also be possible to get it running within Windows.

Dependencies:
 - gcc
 - cmake
 - OpenGL
 - GLUT
 - glfw3
 - glad
 - eigen
 - assimp

Installation guide for Ubuntu (similar for any linux system):
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

FIRST COMPILATION GUIDE

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

QUESTIONS / HELP

As a horrible way of futureproofing, if you happened to be working with or on this project and you have a serious problem, feel free to mail me:
 - ivoblokdoorn@gmail.com
