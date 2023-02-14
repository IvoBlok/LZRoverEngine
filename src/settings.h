#ifndef SETTINGS_H
#define SETTINGS_H
// This is the main settings file meant for adjusting top-down settings. For more technical control over minute behaviour, see the headers in src/lunarEmulation, src/pathplanning, src/segmatch or src/slam


// GENERAL 
// ============================
// defines the amount of frames of path planning based movement are run before another scan of the environment is made and processed
#define FRAMES_PER_SCAN 8
// defines the amount of scans made before a local map is seen as 'full', and passed on to the global map
#define SCANS_PER_LOCAL_MAP 25

// defines if the physics should use real world time, or should use a fixed amount of time between scan iterations. realtime mode requires the computational performance to ensure the iterations per second is high enough for stable performance
#define REALTIME_PHYSICS_ACTIVE false
// defines the time between frames of the main algorithm in seconds. requires REALTIME_PHYSICS_ACTIVE to be false
#define REALTIME_PHYSICS_TIME_PER_FRAME 0.2f 
// defines if the next scan iteration requires a key press to start. In the alternative state the next iteration is immediately started once the current one finishes. Debug rendering is still processed while waiting for the specific key press, making this usefull for debugging
#define START_NEXT_ITERATION_WITH_KEYPRESS_ACTIVE true
// defines the key to press to process the next scan iteration, given that START_NEXT_ITERATION_WITH_KEYPRESS_ACTIVE is true. Should follow the glfw key names
#define START_NEXT_ITERATION_WITH_KEYPRESS_KEY GLFW_KEY_N


// EMULATION ENGINE
// ============================
// defines if synthetic noise will be added to both the pointcloud measurements of the environment and the relative positioning of the rover
#define APPLY_SYNTHETIC_NOISE_TO_EMULATED_SENSORS false
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
#define RESOLUTION_DVG 50
// defines the amount of points that are required to have been stored in a single voxel for it to be considered active, and thus be used in the rest of the navigation solution
#define REQUIRED_POINTCOUNT_TO_BECOME_ACTIVE 2

// defines the max allowed distance squared (in base length units, which in this case is the meter) between the one point and the closest point of a pointcloud. If the distance is larger, the match will not be used in the rest of the ICP algorithm. This is advised to be on the same scale of the width of a voxel
#define MAX_SQUARED_DISTANCE_BETWEEN_ICP_POINT_MATCH 0.04f
// defines the minimum amount of matches found between the source and destination pointcloud that fullfill the requirements, needed to perform the rest of the ICP algorithm. This is to make sure the samples ICP is working with is large enought to be reliable and effective
#define MIN_REQUIRED_ICP_MATCHES 20

#endif