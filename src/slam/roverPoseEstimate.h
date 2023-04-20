#ifndef ROVER_POSE_H
#define ROVER_POSE_H

#include <glm/glm.hpp>
#include <vector>

namespace slam {
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

    // transforms the points retrieved from the rover sensors to the current estimate the class has of where the world space is
    // optionally the transformation from the rover with the engine space can be given so that in the visual environment these nicely overlap
    void transformRoverDepthDataToWorldSpace(std::vector<glm::vec3>& pointcloud, glm::vec3 roverDepthCameraOffset, glm::mat4 initialRoverPose = glm::mat4{1.f}) {
      for (size_t j = 0; j < pointcloud.size(); j++)
      {
        // offset for the sensor position
        pointcloud[j] += roverDepthCameraOffset;

        // transform from relative to absolute purely for debug/visualization reasons
        pointcloud[j] = initialRoverPose * glm::vec4{pointcloud[j], 1.f};

        // transform from rover space to initial pose space, where with 'initial pose space' the 3D space with principal axes aligned with those of the initial pose of the rover
        pointcloud[j] = currentPoseEstimate * glm::vec4{pointcloud[j], 1.f};
      }
    }

  private:
    glm::mat4 currentPoseEstimate = glm::mat4{1.f};
    glm::mat4 previousIMUEstimate = glm::mat4{1.f};
  };
};

#endif