#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#define MAX_SAFE_ANGLE_LUNAR_SURFACE 15.f
#include "../slam/dynamicVoxelGrid.h"

namespace pathplanning {
  const float COS_MAX_SAFE_ANGLE_LUNAR_SURFACE = std::cos(glm::radians(MAX_SAFE_ANGLE_LUNAR_SURFACE));

  // checks if the given voxel has a normal that is too far from perfect upwards. This calculation used is a simplification of the dot product, resulting from the simple form of the perfect upwards vector
  bool isSafeForRoverToTraverse(Voxel* voxel) {
    return glm::abs(glm::normalize(voxel->normal).y) < COS_MAX_SAFE_ANGLE_LUNAR_SURFACE;
  }

  std::vector<glm::vec3> getSafeMovementVoxels(GlobalMap& globalMap) {
    // datablock containing the final data; the centroids of all voxels in the globalMap this function deemed safe to traverse
    std::vector<glm::vec3> safeVoxelCentroids;

    // define some references for an attempt at code readability
    std::vector<MatchingClusterSet>& matchingClusterSets = globalMap.globalClusters.matchingClusterSets;


    if(matchingClusterSets.size() == 0) { return safeVoxelCentroids; }

    // get the groundplane from the given DVG with the largest set of voxels in it. This is assumed to be the one the rover is currently on top of
    // TODO instead get the groundplane cluster which in the horizontal plane contains the rover position
    MatchingClusterSet* largestGroundplaneClusterSet;
    for (auto& cluster : matchingClusterSets) {
      if(cluster.clusters[0].second.ID << 0 && (!largestGroundplaneClusterSet || largestGroundplaneClusterSet->getClusterSetSize() < cluster.getClusterSetSize())) 
        largestGroundplaneClusterSet = &cluster;
    }
    
    // check which points in the cluster are safe to travel, and put the safe ones in a vector to be returned for path planning
    for (size_t i = 0; i < largestGroundplaneClusterSet->clusters.size(); i++)
    {
      for (long& clusterIndex : largestGroundplaneClusterSet->clusters[i].second.voxelIndices) {
        Voxel* voxel = globalMap.DVGs[largestGroundplaneClusterSet->clusters[i].first].getVoxelFromIndex(clusterIndex);
        if(isSafeForRoverToTraverse(voxel))
          safeVoxelCentroids.push_back(voxel->centroid);
      }
    }
  }
}
#endif