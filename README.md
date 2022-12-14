====================================
README FOR OVERALL LZROVERENGINE 
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
are inspired, if not copied from a series of papers, of which a few are listed below:
 - 'SegMap: Segment-based mapping and localization using data-driven descriptors' DOI: 10.1177/0278364919863090
 - 'SegMatch: Segment Based Place Recognition in 3D Point Clouds' DOI: 10.1109/ICRA.2017.7989618
 - 'Incremental Segment-Based Localization in 3D Point Clouds' 

As a horrible way of futureproofing, if you happened to be working with or on this project and you have a serious problem, feel free to mail me:
 - ivoblokdoorn@gmail.com
