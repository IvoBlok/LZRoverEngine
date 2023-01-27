#ifndef ICP_H
#define ICP_H

#include "dynamicVoxelGrid.h"

#include <glm/glm.hpp>

#include <vector>

namespace slam {

  // haven't looked into it too much, but I (Ivo) was planning on basing the ICP implementation on the one presented in the following paper:
  // https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
  glm::mat4 getTransformationEstimateBetweenPointclouds(std::vector<glm::vec3>& sourcePointcloud, DVG& destinationPointcloud) {
    glm::mat4 transformationEstimate = glm::mat4{1.f};


    return transformationEstimate;
  }
};

#endif