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

  std::vector<glm::vec3> subsamplePointclouds(std::vector<std::vector<glm::vec3>>& sourcePointclouds, int desiredPointCount, int sourcePointcloudsPointcount) {
    if(desiredPointCount > sourcePointcloudsPointcount) {
      std::cout << "ERROR: subsamplePointclouds() expected a pointcount smaller then the total pointcount of the given pointcloud\n";
      return std::vector<glm::vec3>{};
    }

    std::vector<glm::vec3> result;
    float index = 0.f;

    // terribly slow way of retrieving the i'th value in a two dimensional array, but the fancy way was too buggy for me rn
    std::vector<glm::vec3> sourcePointcloud;
    for (size_t i = 0; i < sourcePointclouds.size(); i++)
    {
      sourcePointcloud.insert(sourcePointcloud.end(), sourcePointclouds[i].begin(), sourcePointclouds[i].end());
    }
    

    while(result.size() < desiredPointCount) {
      result.push_back(sourcePointcloud[(int)std::round(index)]);

      index += (float)sourcePointcloudsPointcount / (float)desiredPointCount;
    }
    return result;
  }
  

  // haven't looked into it too much, but I (Ivo) was planning on basing the ICP implementation on the one presented in the following paper:
  // https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
  // By now has changed to first doing this, which is quite similar: https://github.com/agnivsen/icp/blob/master/basicICP.py
  glm::mat4 getTransformationEstimateBetweenPointclouds(std::vector<std::vector<glm::vec3>>& sourcePointclouds, DVG& destinationPointcloud) {
    glm::mat4 transformationEstimate = glm::mat4{1.f};
    
    if(destinationPointcloud.voxels.size() == 0)
      return glm::mat4{1.f};
    
    // the rest of the algorithm requires that the source and destination have the same pointcount
    // thus first this needs to be checked, and if necessary, the pointclouds have to be subsampled to the correct size
    int sourceOriginalPointCount = 0;
    for (size_t i = 0; i < sourcePointclouds.size(); i++)
      sourceOriginalPointCount += sourcePointclouds[i].size();
    
    std::vector<glm::vec3> subsampledSourcePointcloud;
    if(sourceOriginalPointCount > destinationPointcloud.voxels.size()) 
    {
      subsampledSourcePointcloud = subsamplePointclouds(sourcePointclouds, destinationPointcloud.voxels.size(), sourceOriginalPointCount);
    }
    else if (sourceOriginalPointCount < destinationPointcloud.voxels.size()) {
      std::cout << "ERROR: NOT IMPLEMENTED YET\n";
      // TODO
      return glm::mat4{1.f};
    }


    // DEBUG
    std::cout << "ICP matrices sizes:" << std::endl;
    std::cout << "subsampledSourcePointcloud: " << sourceOriginalPointCount << " " << destinationPointcloud.voxels.size() << " " << subsampledSourcePointcloud.size() << std::endl;

    // now that the inputs are ready, start by constructing the A and b required to solve the matrix equation which is equivalent to the ICP problem
    Eigen::MatrixXf A{(int)destinationPointcloud.voxels.size(), 6};
    Eigen::VectorXf b{(int)destinationPointcloud.voxels.size()};

    for (size_t i = 0; i < destinationPointcloud.voxels.size(); i++)
    {
      glm::vec3& destNormal = destinationPointcloud.voxels[i].normal;
      glm::vec3& destPosition = destinationPointcloud.voxels[i].centroid;

      glm::vec3& sourcePosition = subsampledSourcePointcloud[i];

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