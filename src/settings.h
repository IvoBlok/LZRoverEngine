#ifndef SETTINGS_H
#define SETTINGS_H

// GENERAL 
// ============================

// defines the amount of frames of path planning based movement are run before another scan of the environment is made and processed
#define FRAMES_PER_SCAN 8
// defines the amount of scans made before a local map is seen as 'full', and passed on to the global map
#define SCANS_PER_LOCAL_MAP 25

// EMULATION ENGINE
// ============================

// defines the resolution of the pointcloud in the horizontal direction relative to the sensor. the resulting pointcloud can have less width then this setting due to there being no surface to see at the edge(s)
#define POINTCLOUD_SCAN_WIDTH 60
// defines the resolution of the pointcloud in the vertical direction relative to the sensor. the resulting pointcloud is likely to have less height then this setting due to there being no surface to see at the edge(s)
#define POINTCLOUD_SCAN_HEIGHT 40


#endif