#ifndef ROVER_POSITION_H
#define ROVER_POSITION_H

#include "../lunarEmulation/LZEngine.h"

#include <glm/glm.hpp>

// all matrices here are defined to be the transformation from the initial state to some other state
class RoverPoseEstimate {
public:
  void modifyCurrentPoseEstimate(glm::mat4 transformation) {
    glm::mat4 currentPose = getMatrixFormOfRoverPose(currentPoseEstimate);

    glm::mat4 newCurrentPose = transformation * currentPose;

    currentPoseEstimate.rotation = glm::mat3{newCurrentPose};
    currentPoseEstimate.translation = glm::vec3{newCurrentPose * glm::vec4{0.f, 0.f, 0.f, 1.f}};
  }

  glm::mat4 getChangeInIMUTransformation(RoverPose& currentIMUEstimate) {
    glm::mat4 previousIMUTransformation = getMatrixFormOfRoverPose(previousIMUEstimate);
    glm::mat4 currentIMUTransformation = getMatrixFormOfRoverPose(currentIMUEstimate);

    return currentIMUTransformation * glm::inverse(previousIMUTransformation);
  }

  void processNewIMUEstimate(RoverPose& currentIMUEstimate) {
    glm::mat4 changeInIMUTransformation = getChangeInIMUTransformation(currentIMUEstimate); 
    previousIMUEstimate = currentIMUEstimate;

    glm::mat4 currentPoseTransformation = changeInIMUTransformation * getMatrixFormOfRoverPose(currentPoseEstimate);

    currentPoseEstimate.rotation = glm::mat3{currentPoseTransformation};
    currentPoseEstimate.translation = glm::vec3{currentPoseTransformation * glm::vec4{0.f, 0.f, 0.f, 1.f}};
  }

  glm::mat4 getCurrentPoseTransformationEstimate() {
    return getMatrixFormOfRoverPose(currentPoseEstimate);
  }
  
  RoverPose getCurrentPoseEstimate() {
    return currentPoseEstimate;
  }

private:
  RoverPose currentPoseEstimate;
  RoverPose previousIMUEstimate;

  glm::mat4 getMatrixFormOfRoverPose(RoverPose& roverPose) {
    glm::mat4 transformation{roverPose.rotation};
    transformation = glm::translate(glm::mat4{1.f}, roverPose.translation) * transformation;
    return transformation;
  }
};

#endif