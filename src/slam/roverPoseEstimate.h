#ifndef ROVER_POSE_H
#define ROVER_POSE_H

#include <glm/glm.hpp>

// all matrices here are defined to be the transformation from the initial state to some other state
class RoverPoseEstimate {
public:
  void modifyCurrentPoseEstimate(glm::mat4 transformation) {
    currentPoseEstimate = transformation * currentPoseEstimate;
  }

  glm::mat4 getChangeInIMUTransformation(glm::mat4& currentIMUEstimate) {
    return currentIMUEstimate * glm::inverse(previousIMUEstimate);
  }

  void processNewIMUEstimate(glm::mat4& currentIMUEstimate) {
    glm::mat4 changeInIMUEstimate = getChangeInIMUTransformation(currentIMUEstimate); 
    previousIMUEstimate = currentIMUEstimate;

    currentPoseEstimate = changeInIMUEstimate * currentPoseEstimate;
  }

  glm::mat4 getCurrentPoseEstimate() {
    return currentPoseEstimate;
  }

private:
  glm::mat4 currentPoseEstimate = glm::mat4{1.f};
  glm::mat4 previousIMUEstimate = glm::mat4{1.f};
};

#endif