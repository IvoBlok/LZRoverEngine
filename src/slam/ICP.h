#ifndef ICP_H
#define ICP_H

#include "dynamicVoxelGrid.h"

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <vector>
#include <cmath>

namespace slam {

  std::vector<glm::vec3> subsamplePointclouds(std::vector<std::vector<glm::vec3>>& sourcePointcloudSet, int desiredPointCount, int pointcountSourcePointcloudSet) {
    if(desiredPointCount > pointcountSourcePointcloudSet) {
      std::cout << "ERROR: subsamplePointclouds() expected a pointcount smaller then the total pointcount of the given pointcloud\n";
      return std::vector<glm::vec3>{};
    }

    std::vector<glm::vec3> result;
    float index = 0.f;

    // terribly slow way of retrieving the i'th value in a two dimensional array, but the fancy way was too buggy for me rn
    std::vector<glm::vec3> sourcePointcloud;
    for (size_t i = 0; i < sourcePointcloudSet.size(); i++)
    {
      sourcePointcloud.insert(sourcePointcloud.end(), sourcePointcloudSet[i].begin(), sourcePointcloudSet[i].end());
    }
    

    while(result.size() < desiredPointCount) {
      result.push_back(sourcePointcloud[(int)std::round(index)]);

      index += (float)pointcountSourcePointcloudSet / (float)desiredPointCount;
    }
    return result;
  }

  // currently this function uses the naive and horribly scaling approach of just iterating over all points in the dvg and checking if it is closer then the ones before
  glm::vec3 findClosestVoxelInDVG(glm::vec3 position, DVG& dvg) {
    float distanceSquaredToClosestVoxel;
    glm::vec3 closestVoxelCentroid;
    for(auto& voxel : dvg.voxels) {
      float squaredDistance = glm::dot(voxel.centroid - position, voxel.centroid - position);

      if(!distanceSquaredToClosestVoxel  || squaredDistance < distanceSquaredToClosestVoxel) {
        distanceSquaredToClosestVoxel = squaredDistance;
        closestVoxelCentroid = voxel.centroid;
      }
    }

    return closestVoxelCentroid;
  }

  std::vector<glm::vec3> findClosestVoxelsInDVGToPointcloudSet(std::vector<std::vector<glm::vec3>>& pointcloudSet, DVG& dvg) {
    std::vector<glm::vec3> matchedPoints;

    // iterate through the nested vector of points in space...
    for(auto& row : pointcloudSet)
      for(auto& element : row)
        matchedPoints.push_back(findClosestVoxelInDVG(dvg.voxels[matchedPoints.size()].centroid, dvg));
    
    return matchedPoints;
  }


  // haven't looked into it too much, but I (Ivo) was planning on basing the ICP implementation on the one presented in the following paper:
  // https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
  // implementation is strongly inspired by the one here: https://github.com/agnivsen/icp/blob/master/basicICP.py
  glm::mat4 getTransformationEstimateBetweenPointclouds(std::vector<std::vector<glm::vec3>>& sourcePointcloudSet, DVG& destinationPointcloud) {
    glm::mat4 transformationEstimate = glm::mat4{1.f};
    
    if(destinationPointcloud.voxels.size() == 0)
      return glm::mat4{1.f};
    
    // find a sufficiently good match for every point in the source pointcloud set within the destination pointcloud
    std::vector<glm::vec3> sourcePointcloudSetMatches = findClosestVoxelsInDVGToPointcloudSet(sourcePointcloudSet, destinationPointcloud);

    // now that the inputs are ready, start by constructing the A and b required to solve the matrix equation which is equivalent to the ICP problem
    Eigen::MatrixXf A{(int)destinationPointcloud.voxels.size(), 6};
    Eigen::VectorXf b{(int)destinationPointcloud.voxels.size()};

    for (size_t i = 0; i < destinationPointcloud.voxels.size(); i++)
    {
      glm::vec3& destNormal = destinationPointcloud.voxels[i].normal;
      glm::vec3& destPosition = destinationPointcloud.voxels[i].centroid;

      glm::vec3& sourcePosition = sourcePointcloudSetMatches[i];

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

    Eigen::MatrixXf pseudoInverseA = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXf transformation = pseudoInverseA * b;

    // go from the 6 elements in a vector describing the transformation to an actual 4x4 transformation matrix
    transformationEstimate = glm::eulerAngleZYX(transformation(2), transformation(1), transformation(0));
    transformationEstimate = glm::translate(transformationEstimate, glm::vec3{transformation(3), transformation(4), transformation(5)});

    return transformationEstimate;
  }
};

#endif