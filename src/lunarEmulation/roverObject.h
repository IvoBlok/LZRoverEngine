#ifndef ROVER_OBJECT_H
#define ROVER_OBJECT_H

#include "model.h"
#include "mesh.h"
#include "lunarSurface.h"

// datablock storing three unit vectors, all orthogonal to each other, encoding the orientation of a sensor
struct SensorPositioning {
  glm::vec3 relativePosition;
  glm::vec3 relativeUp;
  glm::vec3 relativeFront;
};

class Rover : public Model {
public:
  char path[256];
  
  glm::vec3 position{0.f};
  glm::vec3 scale{1.f};
  glm::vec3 front{1.f, 0.f, 0.f};
  glm::vec3 up{0.f, 1.f, 0.f};
  
  std::vector<SensorPositioning> opticalCameraViewMatrices;   
  std::vector<SensorPositioning> depthCameraViewMatrices;   

  // default constructor
  Rover() {}

  // Rover constuctor copied over from Model class
  Rover(string const &path, bool gamma = false) {
    gammaCorrection = gamma;
    
    loadModel(path);
  }
  
  void setupDefaultRover(LunarSurface& surface) {
    if(meshes.size() != 0) {
      return;
    }

    gammaCorrection = gamma;
    loadModel(realpath("../resources/models/rover/LZ_rover_body.obj", path));
    setupSensorConfiguration();

    // start position
    setRoverPosition(glm::vec3{2.f, 2.0f, 5.5f});
    moveAlongSmoothSurface(surface, glm::vec3{0.f});

    setScale(glm::vec3{0.001f});
  }

  void setupSensorConfiguration() {
    // default sensor configurations
    addOpticalCamera(glm::vec3{0.075f, 0.09f, 0.f}, front, up);
    addDepthCamera(glm::vec3{0.075f, 0.09f, 0.f}, front, up);
    //addDepthCamera(glm::vec3{0.f, 0.1f, 0.02f}, glm::rotate(front, glm::radians(-22.5f), up), glm::rotate(up, glm::radians(-5.f), glm::cross(front, up)));
    //addDepthCamera(glm::vec3{0.f, 0.1f, -0.02f}, glm::rotate(front, glm::radians(22.5f), up), glm::rotate(up, glm::radians(-5.f), glm::cross(front, up)));
  }

  // given the relevant model data, this returns the transformation matrix from world space to model space
  glm::mat4 calculateModelMatrix(glm::vec3& position, glm::vec3& front, glm::vec3& up) {
    glm::vec3 right = glm::cross(front, up);
    glm::mat4 model = glm::mat4{1.f};

    model[0] = glm::vec4(glm::normalize(front), 0.f);
    model[1] = glm::vec4(glm::normalize(up), 0.f);
    model[2] = glm::vec4(glm::normalize(right), 0.f);
    model[3] = glm::vec4(position, 1.f);

    return model;
  }

  glm::mat4 calculateModelMatrix() {
    return calculateModelMatrix(position, front, up);
  }

  // calculates the rotation matrix of this rover object relative to world space
  glm::mat3 calculateRotationMatrix() {
    glm::mat3 rot;
    glm::vec3 right = glm::cross(front, up);
    
    rot[0] = glm::normalize(front);
    rot[1] = glm::normalize(up);
    rot[2] = glm::normalize(right);

    return rot;
  }

  // add an optical sensor to the rover at a given position and orientation relative to the zero point of the rover
  void addOpticalCamera(glm::vec3 relativePosition, glm::vec3 relativeFront, glm::vec3 relativeUp) {
    SensorPositioning sensorPos;
    sensorPos.relativePosition = relativePosition;
    sensorPos.relativeFront = relativeFront;
    sensorPos.relativeUp = relativeUp;

    opticalCameraViewMatrices.push_back(sensorPos);
  }

  // add an depth sensor to the rover at a given position and orientation relative to the zero point of the rover
  void addDepthCamera(glm::vec3 relativePosition, glm::vec3 relativeFront, glm::vec3 relativeUp) {
    SensorPositioning sensorPos;
    sensorPos.relativePosition = relativePosition;
    sensorPos.relativeFront = relativeFront;
    sensorPos.relativeUp = relativeUp;

    depthCameraViewMatrices.push_back(sensorPos);
  }

  void translateRover(glm::vec3 translation) {
    position += translation;
  }

  void setRoverPosition(glm::vec3 position) {
    this->position = position;
  }

  glm::vec3 getRoverPosition() {
    return position;
  }

  void setScale(glm::vec3 scale) {
    this->scale = scale;
  }

  glm::vec3 getUp() {
    return up;
  }

  glm::vec3 getFront() {
    return front;
  }

  glm::mat4 getMainRoverModelMatrix() {
    return glm::scale(calculateModelMatrix(), scale);
  }

  void draw(Shader& shader, int FBO = -1) {
    shader.use();
    
    glm::mat4 modelMatrix;
    glm::mat4 roverMatrix = getMainRoverModelMatrix();
    for (size_t i = 0; i < meshes.size(); i++)
    {
      modelMatrix = roverMatrix * meshes[i].getModelMatrix();
      shader.setMat4("model", modelMatrix);

      meshes[i].draw(shader, FBO);
    }
 }

  // Calculate the transformation matrix for the camera-world relative to a given grid, given the position and the angle between the desired front facing vector and the projected positive x-axis vector
  glm::mat4 updateAndGetMainModelMatrixOnSurface(LunarSurface& surface)
  {
    glm::vec3 roverPosition = getRoverPosition();
    roverPosition.x /= surface.baseSurfaceHighDetail.squareSize;
    roverPosition.z /= surface.baseSurfaceHighDetail.squareSize;

    glm::vec3 normal  = -surface.baseSurfaceHighDetail.getNormalOnSmoothSurfaceWithGridPos(roverPosition);
    glm::vec3 normal2 = -surface.baseSurfaceHighDetail.getNormalOnSmoothSurfaceWithGridPos(roverPosition + glm::vec3{1.f, 0.f, 0.f});
    glm::vec3 normal3 = -surface.baseSurfaceHighDetail.getNormalOnSmoothSurfaceWithGridPos(roverPosition + glm::vec3{0.f, 0.f, 1.f});
    glm::vec3 normal4 = -surface.baseSurfaceHighDetail.getNormalOnSmoothSurfaceWithGridPos(roverPosition + glm::vec3{1.f, 0.f, 1.f});

    float deltaX = roverPosition.x - floor(roverPosition.x);
    float deltaZ = roverPosition.z - floor(roverPosition.z);

    normal += deltaX * (normal2 - normal);   // linear interpolation along the floored x at floored z
    normal3 += deltaX * (normal4 - normal3); // linear interpolation along the floored x at floored z + 1
    normal += deltaZ * (normal3 - normal);   // linear interpolation along the non-floored z
    normal = glm::normalize(normal);

    up = -1.f * normal;
    front.y -= (glm::dot(front, up) * up).y;
    front = glm::normalize(front);
    return getMainRoverModelMatrix();
  }

  void moveAlongSmoothSurface(LunarSurface& surface, glm::vec3 translation)
  {
    translateRover(translation);
    position.y = surface.baseSurfaceHighDetail.getHeightOnSmoothSurfaceWithWorldPos(position);
  }
};
#endif