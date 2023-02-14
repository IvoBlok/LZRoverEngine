#ifndef ICP_H
#define ICP_H

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <vector>
#include <cmath>

#include "dynamicVoxelGrid.h"
#include "../settings.h"

namespace slam {

  // currently this function uses the naive and horribly scaling approach of just iterating over all points in the dvg and checking if it is closer then the ones before
  glm::vec3 findClosestVoxelInDVG(glm::vec3 position, DVG& dvg, float& distanceSquared, Voxel& closestVoxel) {
    float distanceSquaredToClosestVoxel = -1.f;

    for(auto& voxel : dvg.voxels) {
      float squaredDistance = std::pow(glm::length(voxel.centroid - position),2);
      if(squaredDistance < distanceSquaredToClosestVoxel || distanceSquaredToClosestVoxel < 0.f) {
        distanceSquaredToClosestVoxel = squaredDistance;
        closestVoxel = voxel;
      }
    }

    distanceSquared = distanceSquaredToClosestVoxel;
    return closestVoxel.centroid;
  }
 
  bool findValidICPMatches(std::vector<std::vector<glm::vec3>>& sourceCloud, DVG& destinationCloud, std::vector<glm::vec3>& filteredSourceCloud, std::vector<Voxel>& filteredDestinationCloud) {
    float distance;
    Voxel matchingVoxel;

    for(auto& sourceCloudRow : sourceCloud) {
      for(auto& sourceCloudElement : sourceCloudRow) {
        findClosestVoxelInDVG(sourceCloudElement, destinationCloud, distance, matchingVoxel);
        if(distance < MAX_SQUARED_DISTANCE_BETWEEN_ICP_POINT_MATCH && matchingVoxel.normal != glm::vec3{0.f}) {
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
  glm::mat4 getTransformationEstimateBetweenPointclouds(std::vector<std::vector<glm::vec3>>& sourcePointcloudSet, DVG& destinationPointcloud) {
    
    if(destinationPointcloud.voxels.size() == 0 || sourcePointcloudSet.size() == 0 || sourcePointcloudSet[0].size() == 0) {
      std::cout << "base matrix returned option 1!\n";
      return glm::mat4{1.f};
    }
    
    // find all matches between points in the source and destination cloud that fullfil the requirements for a match to be valid
    std::vector<glm::vec3> filteredSourceCloud;
    std::vector<Voxel> filteredDestinationVoxelCloud;

    if(!findValidICPMatches(sourcePointcloudSet, destinationPointcloud, filteredSourceCloud, filteredDestinationVoxelCloud)) {
      std::cout << "base matrix returned option 2!\n";
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
      float extraError = std::pow(glm::dot(transformationEstimate * glm::vec4{filteredSourceCloud[i], 1.f} - glm::vec4{filteredDestinationVoxelCloud[i].centroid, 1.f}, glm::normalize(glm::vec4{filteredDestinationVoxelCloud[i].normal, 0.f})), 2); 
      errorAfter += extraError;
    }

    float errorBefore = 0.f;
    for (size_t i = 0; i < filteredDestinationVoxelCloud.size(); i++) {
      errorBefore += std::pow(glm::dot((filteredSourceCloud[i] - filteredDestinationVoxelCloud[i].centroid), glm::normalize(filteredDestinationVoxelCloud[i].normal)), 2);  
    }

    // if for some reason, the error has increased, instead of going down, we want to discard it, so that later iterations will have sufficiently decent matches to be successful
    if(errorBefore < errorAfter) {
      std::cout << "ICP has given an illogical solution, it is discarded and a identity 4x4 matrix is used instead. \n";
      return glm::mat4{1.f};
    }

    return transformationEstimate;
  }
};

#endif