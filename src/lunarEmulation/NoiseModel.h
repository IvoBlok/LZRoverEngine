#ifndef NOISEMODEL_H
#define NOISEMODEL_H

#include <../external/glm/glm.hpp>
#include <utility>

// Approximate the error in IMU pose estimation
class PoseNoiseModel {
private:
  glm::mat3 lastRotation;
  glm::vec3 lastTranslation;

  glm::mat3 lastRotationGuess;
  glm::vec3 lastTranslationGuess;

public:
  PoseNoiseModel(glm::mat3 rot = glm::mat3{1.f}, glm::vec3 trans = glm::vec3{0.f}, glm::mat3 rotGuess = glm::mat3{1.f}, glm::vec3 transGuess = glm::vec3{0.f}) 
  : lastRotation(rot), lastTranslation(trans), lastRotationGuess(rotGuess), lastTranslationGuess(transGuess) {}

  void applyIMUNoise(glm::mat4& transformation) {

    glm::mat3 rotation{transformation};
    glm::vec3 translation = glm::vec3{transformation * glm::vec4{0.f, 0.f, 0.f, 1.f}};

    // exact non drifted delta values
    glm::mat3 deltaRotation = rotation * glm::inverse(lastRotation);
    glm::vec3 deltaTranslation = translation - lastTranslation;

    // update exact state
    lastRotation = rotation;
    lastTranslation = translation;

    // noise model
    // ====================================
    // apply noise on the difference of input since the last time this function was called; This forms the exponential error growth
    // could be improved by aproximating the differential of the parameters, and multiply the error by that, given that the error can be better approximated by  error = d/dt * translation * delta t 
    glm::vec3 randomVec = glm::vec3{2.f * ((float)rand()/(float)RAND_MAX) - 1.f, 2.f * ((float)rand()/(float)RAND_MAX) - 1.f, 2.f * ((float)rand()/(float)RAND_MAX) - 1.f};

    deltaTranslation = deltaTranslation * (0.96f + 0.07f * (float)rand()/(float)RAND_MAX) + glm::length(deltaTranslation) * 0.005f * (randomVec - deltaTranslation * (glm::dot(deltaTranslation, randomVec)/glm::dot(deltaTranslation, deltaTranslation)));
    deltaTranslation = rotation * deltaTranslation;

    // rotation is decomposed into eigenvec and angle, so only a rotation error can be made across the one variable
    // eig decomp
    if(deltaRotation - glm::transpose(deltaRotation) == glm::mat3{0.f}) { std::cout << "Eigenvector decomposition Exception \n"; return; }
    glm::vec3 eigenVec = glm::normalize(glm::vec3{
      deltaRotation[1][2] - deltaRotation[2][1],
      deltaRotation[2][0] - deltaRotation[0][2],
      deltaRotation[0][1] - deltaRotation[1][0]  
    });
    // get angle by which the 3x3 Identity matrix is rotated around 'eigenVec' to get 'changeInRealPoseSinceLast'
    float trace = deltaRotation[0][0] + deltaRotation[1][1] + deltaRotation[2][2];
    float angle = std::acos((trace - 1.f)/2.f);

    angle *= 0.98f + 0.032f * (float)rand()/(float)RAND_MAX;
    eigenVec = glm::normalize(eigenVec - 0.001f * randomVec);   
    // calculate drifted rotation matrrix
    deltaRotation = glm::rotate(angle, eigenVec);

    // calculate new estimate
    rotation = deltaRotation * lastRotationGuess;
    translation = glm::inverse(rotation) * deltaTranslation + lastTranslationGuess;

    transformation = glm::mat4{rotation};
    transformation = glm::translate(glm::mat4{1.f}, translation) * transformation;

    // update estimates state
    lastRotationGuess = rotation;
    lastTranslationGuess = translation;
  }  
};

// Approximate the error curves of TOF/Lidar 
class PointcloudNoiseModel {
public:
  bool applyPointNoise(glm::vec3& point) {
    return glm::length(point) > 2.f;
  }
};

#endif