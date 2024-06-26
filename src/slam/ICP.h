#ifndef ICP_H
#define ICP_H

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <vector>
#include <cmath>

#include "dynamicVoxelGrid.h"
#include "roverPoseEstimate.h"
#include "../slam/KDTree.h"
#include "../settings.h"


namespace slam {
  // searches through 3D tree for the closest voxel
  void findClosestVoxelInDVG(glm::vec3& sourcePoint, DVG& dvg, Voxel& closestVoxel) {
    closestVoxel = *dvg.tree->nearest(sourcePoint);
  }
 
  bool findValidICPMatches(std::vector<std::vector<glm::vec3>>& sourceCloud, DVG& destinationCloud, std::vector<glm::vec3>& filteredSourceCloud, std::vector<Voxel>& filteredDestinationCloud) {
    Voxel matchingVoxel;

    for(auto& sourceCloudRow : sourceCloud) {
      for(glm::vec3& sourceCloudElement : sourceCloudRow) {
        findClosestVoxelInDVG(sourceCloudElement, destinationCloud, matchingVoxel);
        if(glm::dot(matchingVoxel.normal, matchingVoxel.normal) > 0
        && destinationCloud.tree->distance() < MAX_SQUARED_DISTANCE_BETWEEN_ICP_POINT_MATCH) {
          filteredSourceCloud.push_back(sourceCloudElement);
          filteredDestinationCloud.push_back(matchingVoxel);
        }
      }
    }
    return filteredSourceCloud.size() >= MIN_REQUIRED_ICP_MATCHES;
  }

  // haven't looked into it too much, but I (Ivo) was planning on basing the ICP implementation on the one presented in the following paper:
  // https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
  // implementation is strongly inspired by the one here: https://github.com/agnivsen/icp/blob/master/basicICP.py
  glm::mat4 getTransformationEstimateBetweenPointclouds(std::vector<std::vector<glm::vec3>>& sourcePointcloudSet, DVG& destinationPointcloud, float& errorChange) {
    
    if(destinationPointcloud.voxels.size() == 0 || sourcePointcloudSet.size() == 0 || sourcePointcloudSet[0].size() == 0) {
      errorChange = 0;
      return glm::mat4{1.f};
    }
    
    // find all matches between points in the source and destination cloud that fullfil the requirements for a match to be valid
    std::vector<glm::vec3> filteredSourceCloud;
    std::vector<Voxel> filteredDestinationVoxelCloud;

    if(!findValidICPMatches(sourcePointcloudSet, destinationPointcloud, filteredSourceCloud, filteredDestinationVoxelCloud)) {
      errorChange = 0;
      return glm::mat4{1.f};
    }
    
    // now that the inputs are ready, start by constructing the A and b required to solve the matrix equation which is equivalent to the ICP problem
    Eigen::MatrixXf A{(int)filteredDestinationVoxelCloud.size(), 6};
    Eigen::VectorXf b{(int)filteredDestinationVoxelCloud.size()};

    for (size_t i = 0; i < filteredDestinationVoxelCloud.size(); i++)
    {
      glm::vec3 destNormal = glm::normalize(filteredDestinationVoxelCloud[i].normal);
      glm::vec3& destPosition = filteredDestinationVoxelCloud[i].centroid;

      glm::vec3& sourcePosition = filteredSourceCloud[i];

      float a1 = (destNormal.z * sourcePosition.y) - (destNormal.y * sourcePosition.z);
      float a2 = (destNormal.x * sourcePosition.z) - (destNormal.z * sourcePosition.x);
      float a3 = (destNormal.y * sourcePosition.x) - (destNormal.x * sourcePosition.y);
  	  
      Eigen::VectorXf _A{6};
      _A(0) = a1;
      _A(1) = a2;
      _A(2) = a3;
      _A(3) = destNormal.x;
      _A(4) = destNormal.y;
      _A(5) = destNormal.z;

      float _b = (destNormal.x * destPosition.x) + (destNormal.y * destPosition.y) + (destNormal.z * destPosition.z) 
        - (destNormal.x * sourcePosition.x) - (destNormal.y * sourcePosition.y) - (destNormal.z * sourcePosition.z);

      A.row(i) = _A;
      b(i) = _b;
    }

    // solve the matrix equation ...
    Eigen::MatrixXf pseudoInverseA = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXf transformation = pseudoInverseA * b;

    // go from the 6 elements in a vector describing the transformation to an actual 4x4 transformation matrix
    glm::mat4 transformationEstimate = glm::eulerAngleZYX(transformation(2), transformation(1), transformation(0));
    transformationEstimate = glm::translate(glm::mat4{1.f}, glm::vec3{transformation(3), transformation(4), transformation(5)}) * transformationEstimate;

    // for debugging reasons, calculate the error this is supposed to have minimized
    float errorAfter = 0.f;
    for (size_t i = 0; i < filteredDestinationVoxelCloud.size(); i++) {
      float extraError = std::pow(glm::dot((glm::vec3{transformationEstimate * glm::vec4{filteredSourceCloud[i], 1.f}} - filteredDestinationVoxelCloud[i].centroid), glm::normalize(filteredDestinationVoxelCloud[i].normal)), 2); 
      errorAfter += extraError;
    }

    float errorBefore = 0.f;
    for (size_t i = 0; i < filteredDestinationVoxelCloud.size(); i++) {
      errorBefore += std::pow(glm::dot((filteredSourceCloud[i] - filteredDestinationVoxelCloud[i].centroid), glm::normalize(filteredDestinationVoxelCloud[i].normal)), 2);  
    }

    // if for some reason, the error has increased, instead of going down, it is likely to be due to the fact that the optimal transformation is just the 4x4 I matrix.
    if(errorBefore < errorAfter) {
      if(PRINT_ICP_DEBUG_INFO) {std::cout << "4x4 I matrix returned. \nError increased by: " << errorAfter - errorBefore << " \ntotal error before: " << errorBefore << "\n";}
      errorChange = 0;
      return glm::mat4{1.f};
    }
    
    errorChange = errorBefore - errorAfter;
    return transformationEstimate;
  }

  void ICP(std::vector<std::vector<glm::vec3>>& pointclouds, DVG& localMap, RoverPoseEstimate& roverPoseEstimate) {
    // run the iterative process of ICP as long as the error keeps decreasing fast enough
    float lastErrorChange = -1.f;
    while((lastErrorChange == -1.f) || lastErrorChange > MIN_ERROR_DECREASE_PER_ICP_ITERATION) {
      // Calculate the transformation from the new pointcloud with the estimated pose applied, to the current local map with partial ICP. 
      // The resulting matrix is the result of noise in mainly the Pose measuring device. ICP inherently also doesn't get to the exact answer, but it should be enough for this application
      glm::mat4 transformationEstimate = getTransformationEstimateBetweenPointclouds(pointclouds, localMap, lastErrorChange);
      

      if(transformationEstimate == glm::mat4{1.f}) {
        lastErrorChange = MIN_ERROR_DECREASE_PER_ICP_ITERATION * 0.5f; // handwavy way of ensuring that this will be the last ICP iteration in this situation
      } else {
        // update where the software thinks the rover is at, thus improving the accuracy of the localization
        roverPoseEstimate.modifyCurrentPoseEstimate(transformationEstimate);

        // Apply the newly found transformation matrix, and push the new pointcloud to the local DVG
        for (size_t i = 0; i < pointclouds.size(); i++){
          for (size_t j = 0; j < pointclouds[i].size(); j++) {
            pointclouds[i][j] = transformationEstimate * glm::vec4{pointclouds[i][j], 1.f};
          }
        }
        if(PRINT_ICP_DEBUG_INFO) {std::cout << "error change: " << lastErrorChange << std::endl;}
      }
      if(PRINT_ICP_DEBUG_INFO) {std::cout << "Optimal transformation proposed by last ICP iteration: \n" << glm::to_string(transformationEstimate) << std::endl;}
    }
  }
};

#endif