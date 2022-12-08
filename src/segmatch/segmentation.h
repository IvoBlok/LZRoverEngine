#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "../slam/dynamicVoxelGrid.h"
#include "normalEstimator.h"

#include <vector>
#include <map>
#include <list>

#define CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD 5.0f
#define MAX_ANGLE_TO_UPWARDS_THRESHOLD 0.95f // effectively the cosine of the max allowed angle between upwards world vec and the ground plane normal

std::vector<glm::vec3> visualizationNormals;

void growClusters(DVG& dvg, NormalEstimator& normalEstimator, std::vector<long> newActiveVoxelIndices) {

  // filter new active voxels for points that'll be effective as seeds to grow the ground plane from
  for (size_t i = 0; i < newActiveVoxelIndices.size(); i++) {
    Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]);
    
    // check if it's somewhat close to facing upwards
    if(glm::abs(glm::dot(glm::normalize(voxel->normal), glm::vec3{0.f, 1.f, 0.f})) > MAX_ANGLE_TO_UPWARDS_THRESHOLD) {
      std::vector<Voxel> neighbours = dvg.getNeighbours(newActiveVoxelIndices[i]);
      
      // check the neighbours as a crude way of checking the curvature, based on the idea that obstacles will have significantly smaller curvatures (with respect to the ground plane)
      bool neighboursPassed = true;
      for (size_t j = 0; j < neighbours.size(); j++) {
        if(glm::abs(glm::dot(glm::normalize(neighbours[j].normal), voxel->normal)) < MAX_ANGLE_TO_UPWARDS_THRESHOLD / 2.f) {
          neighboursPassed = false;
          break;
        }
      }
      
      if(neighboursPassed) {
        voxel->clusterID = 1;
      }
    }
  }

  for (size_t i = 0; i < newActiveVoxelIndices.size(); i++)
  {
    // retrieve the neighbours
    std::vector<Voxel> neighbours = dvg.getNeighbours(newActiveVoxelIndices[i]);
    Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]);

    bool partOfGroundPlane = false;
  
    // grow ground plane from the selected seeds if it has a neighbour with close enough normals
    for (size_t j = 0; j < neighbours.size(); j++)
    {
      if(neighbours[j].clusterID == 0) { continue; }

      float angle = std::acos(glm::dot(glm::normalize(neighbours[j].normal), glm::normalize(voxel->normal))) * (360.f / 6.28f);
      
      if(angle < CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD) {
        partOfGroundPlane = true;
        break;
      }
    }

    // Filter out points that are likely to be too steep to climb for the rover
    if(partOfGroundPlane) {
      // check if the normal itself isn't too far from perfect upwards
      partOfGroundPlane = glm::abs(glm::dot(glm::normalize(voxel->normal), glm::vec3{0.f, 1.f, 0.f})) > MAX_ANGLE_TO_UPWARDS_THRESHOLD;
    }

    // set the clusterIndex appropriately
    if(partOfGroundPlane) {
        voxel->clusterID = 1;
    }
  }
}

#endif