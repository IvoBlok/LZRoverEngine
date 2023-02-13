#ifndef SETTINGS_H
#define SETTINGS_H
// This is the main settings file meant for adjusting top-down settings. For more technical control over minute behaviour, see the headers in src/lunarEmulation, src/pathplanning, src/segmatch or src/slam

// GENERAL 
// ============================
// defines the amount of frames of path planning based movement are run before another scan of the environment is made and processed
#define FRAMES_PER_SCAN 8
// defines the amount of scans made before a local map is seen as 'full', and passed on to the global map
#define SCANS_PER_LOCAL_MAP 25
// defines if synthetic noise will be added to both the pointcloud measurements of the environment and the relative positioning of the rover
#define APPLY_SYNTHETIC_NOISE_TO_EMULATED_SENSORS false


// EMULATION ENGINE
// ============================
// defines the resolution of the pointcloud in the horizontal direction relative to the sensor. the resulting pointcloud can have less width then this setting due to there being no surface to see at the edge(s)
#define POINTCLOUD_SCAN_WIDTH 60
// defines the resolution of the pointcloud in the vertical direction relative to the sensor. the resulting pointcloud is likely to have less height then this setting due to there being no surface to see at the edge(s)
#define POINTCLOUD_SCAN_HEIGHT 40
// defines if the pointcloud measurements of the emulated environment should be stored to some form of accessible permanent storage for potential further analysis / debugging
#define EXPORTPOINTCLOUD false
// defines if the optical measurements (images) of the emulated environment should be stored to some form of accessible permanent storage for potential further analysis / debugging
#define EXPORTIMAGE false
// defines if the location of the sensors should be added to the pointcloud returned from taking a measurement. Be warned, this might mess with the other navigation steps and is intended to debug the emulation engine
#define DEBUG_CAMERA_ORIENTATIONS false

// SLAM
// ============================
// defines the amount of voxels per unit length, or in other words, this setting to the third power is the max possible amount of voxels in a unit volume, which in this case is set to be roughly 1 m^3
#define RESOLUTION_DVG 40
// defines the amount of points that are required to have been stored in a single voxel for it to be considered active, and thus be used in the rest of the navigation solution
#define REQUIRED_POINTCOUNT_TO_BECOME_ACTIVE 2

#endif