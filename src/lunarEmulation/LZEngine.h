#ifndef LZ_ENGINE_H
#define LZ_ENGINE_H

// external libraries
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <../external/glm/glm.hpp>
#include <../external/glm/gtc/matrix_transform.hpp>
#include <../external/glm/gtc/type_ptr.hpp>
#include <../external/glm/gtx/rotate_vector.hpp>
#include <../external/Perlin.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <../external/stb_image_write.h>

// internal
#include "shaders/shader.h"
#include "user_camera.h"

#include "roverObject.h"
#include "lunarSurface.h"
#include "dynamicVoxelGridMesh.h"

#include "NoiseModel.h"

// standard library
#include <iostream>
#include <stdlib.h>

Shader depthShader{};
Shader opticalShader{};
Shader pointShader{};
Shader lineShader{};

UserCamera camera{};

// timing
float debugDeltaTime = 0.0f;
float lastFrame = 0.0f;

// data block that stores the difference between state A and B of the rover in world space
// =============================================
// glm::mat3 rotation : 3x3 rotation matrix from state A to B 
// glm::vec3 translation : translation of the zero point from state A to B, relative to the triad defined in 'rotation'
struct RoverPose {
  glm::mat3 rotation = glm::mat3{1.f};
  glm::vec3 translation = glm::vec3{0.f};
};

// data block that is returned from the engine if a request for a pointcloud is issued. 
// roverUp and roverFront together encode the 3D orientation of the rover. 
// =============================================
// std::vector<glm::vec3> pointcloud : vector of 3D points that hit the simulated lunar surface. starts at the left bottom corner, and goes up row wise
// RoverPose pose : pose estimate of the rover according to the IMU at the time of retrieving the data in the package
struct RoverDepthDataPackage {
  std::vector<std::vector<glm::vec3>> pointclouds;
  RoverPose pose;
};

// data block that is returned from the engine if a request for a RGB image is issued.
// roverUp and roverFront together encode the 3D orientation of the rover. 
// =============================================
// std::vector<float*> opticalImages : vector of array of floats storing the images of the cameras on the rover. for every value in the vector, the float array starts at the left bottom corner, and goes up row wise
// std::vector<unsigned int> opticalImageWidths : amount of pixels per row of the image for every image in opticalImages
// std::vector<unsigned int> opticalImageHeights : amount of pixels per column of the image for every image in opticalImages
// RoverPose pose : pose estimate of the rover according to the IMU at the time of retrieving the data in the package
struct RoverImageDataPackage {
  std::vector<float*> opticalImages;
  std::vector<unsigned int> opticalImageWidths;
  std::vector<unsigned int> opticalImageHeights;

  RoverPose pose;
};

// class managing the emulation of all required physical components of the mission (starting from deployment).
// Can:
//  * Generate a procedural lunar surface
//  * Calculate the image the rover cameras would see
//  * Calculate the points the rover TOFs / Lidars would see
//  * Render the environment (in between calculation cycles) to a debug window with a separate user controllable camera
//
// Planned features:
//  * Simulate the rovers movement in a physically accurate way over the surface
//  * Simulate the noisy localization of the rover
//
// Currently limited to a single rover, with a theoretically unlimited amount of TOF / Image sensors
// =============================================
// LunarSurface lunarSurfaceObject : class instance containing all lunar features and a variety of functions to generate and use the terrain.
// Rover roverObject : class instance containing rover specific information and functions for manipulating the rover. 
// DVGMesh DVGObject : class instance containing DVG specific functions to render a dynamic voxel grid
// std::vector<Model> restObject : vector of models. This is meant to hold all leftover models that are in the scene. This should be empty most of the time. envisioned mainly for debugging
class LZEngine {
public:
  LunarSurface lunarSurfaceObject{};
  Rover roverObject{};
  std::vector<Model> restObjects;

  // debug rendering object data
  std::vector<DVGMesh> DVGObjects;
  std::vector<LineMesh> linesObjects;

  GLFWwindow* window;

  // trajectory estimation
  RoverPose IMUPoseEstimate;
  RoverPose initialRealPose;
  RoverPose realPose;
  PoseNoiseModel poseNoiseModelObj;
  PointcloudNoiseModel pclNoiseModelObj;

  // settings
  const unsigned int SCR_WIDTH = 800;
  const unsigned int SCR_HEIGHT = 600;

  const unsigned int POINTCLOUD_WIDTH;
  const unsigned int POINTCLOUD_HEIGHT;

  const float NEAR_PLANE = 0.001f;
  const float FAR_PLANE = 25.f;

  // max distance the depth buffer is supported to see per direction
  const float MAX_X = 1000.f;
  const float MAX_Y = 1000.f;
  const float MAX_Z = 1000.f;

  // default constructor of LZEngine
  // =============================================
  LZEngine(
    unsigned int pointcloudWidth = 400, 
    unsigned int pointcloudHeight = 300
    ) : 
    POINTCLOUD_WIDTH(pointcloudWidth), 
    POINTCLOUD_HEIGHT(pointcloudHeight) {}

  // function responsible for transforming the image of 3d world positions for every rendered pixel in clip space. 
  // to a vector of 3D world positions in world space of a desired end resolution. points are relative to the rover depth sensor
  // desired resultion given by POINTCLOUD_WIDTH and POINTCLOUD_HEIGHT settings
  // =============================================
  // float* image : float array storing the world positions in clip space for every pixel in the window
  // unsigned int sensorID : ID of the depth sensor used for the image. Equals the index of the depth sensor in the depth sensor vector of the rover instance
  // returns std::vector<glm::vec3> : a pointcloud, consisting of avector of 3d points containing the world positions in world space that were seen by the emulated sensor
  std::vector<glm::vec3> getPointCloud(float* image, unsigned int sensorID = 0) {
    std::vector<glm::vec3> pointcloud;

    glm::vec3 roverPosition = roverObject.getRoverPosition();
    glm::mat3 WorldToRoverMatrix = glm::inverse(roverObject.calculateRotationMatrix());

    float resXFactor = (float)SCR_WIDTH / (float)POINTCLOUD_WIDTH;
    float resYFactor = (float)SCR_HEIGHT / (float)POINTCLOUD_WIDTH;

    for (float x = 0; x < POINTCLOUD_WIDTH; x++)
    {
      for (float y = 0; y < POINTCLOUD_WIDTH; y++)
      {
        unsigned int xi = (unsigned int)floor(x * resXFactor);
        unsigned int yi = (unsigned int)floor(y * resYFactor);

        // retrieve point position in world space from the 3 channel buffer mapped to a 1d array, each channel corresponding to one of the independent axes of the world
        glm::vec3 worldPos = glm::vec3{image[(yi * SCR_WIDTH + xi) * 3], image[(yi * SCR_WIDTH + xi) * 3 + 1], image[(yi * SCR_WIDTH + xi) * 3 + 2]};
        
        // reverse the transformation applied for data transfer from GPU memory to CPU memory
        worldPos -= 0.5f;
        worldPos *= 2.f;
        worldPos *= glm::vec3{MAX_X, MAX_Y, MAX_Z};

        // filter out those points with the default value of the depth buffer
        if(abs(worldPos.x) >= .99f * MAX_X || abs(worldPos.z) >= .99f * MAX_Z){
          continue;
        }

        // translate the world position of the point into relative to the rover zero point 
        worldPos -= roverPosition;

        // for rock training data this removes the points below a arbitrary plane
        glm::vec3 planeNormal = glm::vec3{0.f, 1.f, 0.f} + glm::vec3{(float)rand()/(float)RAND_MAX * 0.2f, 0.f, (float)rand()/(float)RAND_MAX * 0.2f};
        bool aboveGroundPlane = glm::dot(worldPos - glm::vec3{1.f, 0.f, 0.f}, planeNormal) > 0.f;  

        // apply noise (and do depth check)
        if(!pclNoiseModelObj.applyPointNoise(worldPos) && aboveGroundPlane) { pointcloud.push_back(worldPos); }
      }
    }

    return pointcloud;
  }

  // function responsible for initializing the engine.
  // Does the following:
  // * Initialize the GLFW window
  // * Sets rendering settings
  // * Intialized shader programs
  // * Sets up opengl Buffers
  // * Sets up memory for later use in the LZEngine
  // * Generates the lunar surface
  // =============================================
  void startEngine() {
    /* #region set up window */
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LZRoverEngine", NULL, NULL);
    if (!window)
    {
      std::cout << "Failed to create GLFW window" << std::endl;
      glfwTerminate();
      return;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
      std::cout << "Failed to initialize GLAD" << std::endl;
      return;
    }

    glfwSwapInterval(0);
    stbi_set_flip_vertically_on_load(true);
    stbi_flip_vertically_on_write(true);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);  
    glEnable(GL_PROGRAM_POINT_SIZE);
    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    /* #endregion */

    // build and compile our shader program
    depthShader = Shader{"../src/lunarEmulation/shaders/depthShader.vs", "../src/lunarEmulation/shaders/depthShader.fs"};
    opticalShader = Shader{"../src/lunarEmulation/shaders/opticalShader.vs", "../src/lunarEmulation/shaders/opticalShader.fs"};
    pointShader = Shader{"../src/lunarEmulation/shaders/pointShader.vs", "../src/lunarEmulation/shaders/pointShader.fs"};
    lineShader = Shader{"../src/lunarEmulation/shaders/lineShader.vs", "../src/lunarEmulation/shaders/lineShader.fs"};

    // init depth image buffers
    depthImage = new float[SCR_WIDTH * SCR_HEIGHT * 3];

    opticalImageR = new float[SCR_WIDTH * SCR_HEIGHT];
    opticalImageG = new float[SCR_WIDTH * SCR_HEIGHT];
    opticalImageB = new float[SCR_WIDTH * SCR_HEIGHT];

    // setup depth shader variables
    depthShader.use();
    depthShader.setFloat("maxX", MAX_X);
    depthShader.setFloat("maxY", MAX_Y);
    depthShader.setFloat("maxZ", MAX_Z);

    // setup texture to render to with custom framebuffer
    glGenFramebuffers(1, &FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, FBO);

    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
    
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, depthTexture, 0);

    glGenTextures(1, &depthDepthTexture);
    glBindTexture(GL_TEXTURE_2D, depthDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SCR_WIDTH, SCR_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthDepthTexture, 0);
    
    GLenum drawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, drawBuffers);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) { std::cout << "issue setting up framebuffer \n";}

    glBindFramebuffer(GL_FRAMEBUFFER, 0);  

    // generate 'rover'
    roverObject.setupDefaultRover(lunarSurfaceObject);

    // initialize IMU variables
    IMUPoseEstimate.rotation = glm::mat3{1.f};
    IMUPoseEstimate.translation = glm::vec3{0.f};

    initialRealPose.rotation = glm::inverse(roverObject.calculateRotationMatrix());
    initialRealPose.translation = roverObject.position;

    realPose.rotation = initialRealPose.rotation;
    realPose.translation = initialRealPose.translation;

    // initialize noise models
    poseNoiseModelObj = PoseNoiseModel{IMUPoseEstimate.rotation, IMUPoseEstimate.translation, IMUPoseEstimate.rotation, IMUPoseEstimate.translation};

    // Set debug camera
    camera.Position = glm::vec3{1.f, 1.f, 5.f};
  }

  // function responsible for cleaning up the used memory.
  // =============================================
  void stopEngine() {
    delete[] depthImage;
    
    delete[] opticalImageR;
    delete[] opticalImageG;
    delete[] opticalImageB;

    glfwTerminate();
  }

  // function responsible for retrieving the pointcloud information and storing it in a given datablock
  // if EXPORTPOINTCLOUD is defined, the resulting pointcloud is stored as a .csv file in the directory of the executable
  // =============================================
  // RoverDepthDataPackage& data : datablock where the newly calculated TOF / LIDAR sensor data is to be stored
  void getDepthDataPackage(RoverDepthDataPackage& data) {
    if(glfwWindowShouldClose(window)) { return; }
    data.pointclouds.clear();

    depthShader.use();
    depthShader.setMat4("projection", projection);

    glm::mat4 roverModelMatrixUnscaled = roverObject.calculateModelMatrix(roverObject.position, roverObject.front, roverObject.up);
    glm::mat4 viewMatrixInWorldCoord;

    // bind the frame buffer for the depth calculation pipeline
    glBindFramebuffer(GL_FRAMEBUFFER, FBO);

    for (size_t i = 0; i < roverObject.depthCameraViewMatrices.size(); i++)
    {
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      // calculate camera orientation and position in world coords
      glm::vec3 positionCam = roverModelMatrixUnscaled * glm::vec4{roverObject.depthCameraViewMatrices[i].relativePosition, 1.f};
      glm::vec3 frontCam = glm::normalize(roverModelMatrixUnscaled * glm::vec4{roverObject.depthCameraViewMatrices[i].relativeFront, 0.f});
      glm::vec3 upCam = glm::normalize(roverModelMatrixUnscaled * glm::vec4{roverObject.depthCameraViewMatrices[i].relativeUp, 0.f});

      //calculate view matrix from the previously found camera triad in world coordinates
      viewMatrixInWorldCoord = glm::lookAt(positionCam, positionCam + frontCam, upCam);
      depthShader.setMat4("view", viewMatrixInWorldCoord);
      
      // update and draw grid
      lunarSurfaceObject.draw(depthShader);

      // extract resulting pointcloud and rover information to be returned
      glBindTexture(GL_TEXTURE_2D, depthTexture);
      glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, depthImage);

      data.pointclouds.push_back(getPointCloud(depthImage, i));

      #ifdef DEBUG_CAMERA_ORIENTATIONS
        data.pointclouds.back().push_back(positionCam);
        data.pointclouds.back().push_back(positionCam + 0.1f * upCam);
        data.pointclouds.back().push_back(positionCam + 0.1f * frontCam);
      #endif
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
    // bind the default frame buffer object again
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // set non-unique request data
    data.pose.rotation = IMUPoseEstimate.rotation;
    data.pose.translation = IMUPoseEstimate.translation;

    #ifdef EXPORTPOINTCLOUD
      pointcloudFileIndex++;
      std::string pointcloudFileName = "pointcloud" + to_string(pointcloudFileIndex) + ".csv";
      std::ofstream pointcloudOutStream(pointcloudFileName);
      
      for (size_t i = 0; i < data.pointclouds.size(); i++)
      {
        for(glm::vec3& point : data.pointclouds[i]) {
          pointcloudOutStream << point.x << "," << point.y << "," << point.z << "\n";
        }
      }
    #endif

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // function responsible for retrieving the optical image and storing it in a given datablock
  // if EXPORTIMAGE is defined, the resulting image is stored as a .png file in the directory of the executable
  // =============================================
  // RoverImageDataPackage& data : datablock where the newly calculated camera sensor data is to be stored
  void getImageDataPackage(RoverImageDataPackage& data) {
    if(glfwWindowShouldClose(window)) { return; }
    
    opticalShader.use();

    // lighting
    opticalShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    opticalShader.setVec3("lightDir", lightDir);

    opticalShader.setMat4("projection", projection);

    glm::mat4 roverModelMatrixUnscaled = roverObject.calculateModelMatrix(roverObject.position, roverObject.front, roverObject.up);
    glm::mat4 viewMatrixInWorldCoord;
   
    for (size_t i = 0; i < roverObject.opticalCameraViewMatrices.size(); i++)
    {
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // calculate camera orientation and position in world coords
      glm::vec3 positionCam = roverModelMatrixUnscaled * glm::vec4{roverObject.opticalCameraViewMatrices[i].relativePosition, 1.f};
      glm::vec3 frontCam = glm::normalize(roverModelMatrixUnscaled * glm::vec4{roverObject.opticalCameraViewMatrices[i].relativeFront, 0.f});
      glm::vec3 upCam = glm::normalize(roverModelMatrixUnscaled * glm::vec4{roverObject.opticalCameraViewMatrices[i].relativeUp, 0.f});
      //calculate view matrix from the previously found camera triad in world coords
      viewMatrixInWorldCoord = glm::lookAt(positionCam, positionCam + frontCam, upCam);
      
      opticalShader.setMat4("view", viewMatrixInWorldCoord);

      // update and draw grid
      lunarSurfaceObject.draw(opticalShader);

      // update and draw rover
      roverObject.draw(opticalShader);

      // update and draw rest objects
      for (size_t i = 0; i < restObjects.size(); i++)
      {
        model = restObjects[i].meshes[0].getModelMatrix();
        opticalShader.setMat4("model", model);
        restObjects[i].draw(opticalShader);
      }

      // retrieve image(s) and rover information to be returned
      // just use red channel to retrieve the optical image. Given that the output image is required to be grayscale, the green and blue values can be ignored and simplifies the image preprocessing
      glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_RED, GL_FLOAT, opticalImageR);
      glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_GREEN, GL_FLOAT, opticalImageG);
      glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_BLUE, GL_FLOAT, opticalImageB);

      data.opticalImages.push_back(new float[SCR_WIDTH * SCR_HEIGHT * 3]);

      for (int i = 0; i < SCR_WIDTH * SCR_HEIGHT; i++)
      {
        data.opticalImages.back()[i * 3] = opticalImageR[i];
        data.opticalImages.back()[i * 3 + 1] = opticalImageG[i];
        data.opticalImages.back()[i * 3 + 2] = opticalImageB[i];
      }

      data.opticalImageWidths.push_back(SCR_WIDTH);
      data.opticalImageHeights.push_back(SCR_HEIGHT);

      glfwSwapBuffers(window);
      glfwPollEvents();
    }

    // set non-sensor request data
    data.pose.rotation = IMUPoseEstimate.rotation;
    data.pose.translation = IMUPoseEstimate.translation;

    #ifdef EXPORTIMAGE
      imageFileIndex++;
      std::string opticalImageFileName = "opticalImage" + to_string(imageFileIndex) + ".png";

      unsigned char* opticalImageReformed = new unsigned char[SCR_WIDTH * SCR_HEIGHT * 3];
      for (int i = 0; i < SCR_WIDTH * SCR_HEIGHT * 3; i++)
      {
        opticalImageReformed[i] = 256 * data.opticalImage[i];
      }
      
      stbi_write_png(opticalImageFileName.c_str(), SCR_WIDTH, SCR_HEIGHT, 3, opticalImageReformed, 0);
    #endif
  }

  // function responsible for updating the Rover Pose Estimate to the current state. 
  // This function includes the drifting effect of numerically aproximating an integral.
  // =============================================
  void updateIMUEstimate(bool applyNoise = true) {  
    // update realPose
    realPose.rotation = glm::inverse(roverObject.calculateRotationMatrix());
    realPose.translation = roverObject.getRoverPosition();

    // update exact IMU
    IMUPoseEstimate.rotation = realPose.rotation * glm::inverse(initialRealPose.rotation);
    IMUPoseEstimate.translation = realPose.translation - initialRealPose.translation;

    // noise
    if(applyNoise) { poseNoiseModelObj.applyIMUNoise(IMUPoseEstimate.rotation,IMUPoseEstimate.translation); }
  }

  // function responsible for returning a copy of the 'initialRealPose' variable. 
  // This is usefull for the debugging visual environment, so that the relative measurements can be offset to overlap with the actual world.
  // =============================================
  // returns RoverPose : copy of the 'initialRealPose' variable
  RoverPose getInitialRealPose() {
    return initialRealPose;
  }

  // function responsible for updating the debugDeltaTime variable
  // =============================================
  // returns float : updated debugDeltaTime value
  float updateDeltaTime() {
    // time logic
    debugDeltaTime = static_cast<float>(glfwGetTime()) - lastFrame;
    lastFrame += debugDeltaTime;
    return debugDeltaTime;
  }

  // function responisble for moving and orientating the rover in a given direction
  // =============================================
  // glm::vec3 direction : vector in the direction the rover should move
  // float speed : desired speed by which the rover should move in the given 'direction'
  void moveRoverInDirection(glm::vec3 direction, float speed) {
    roverObject.moveAlongSmoothSurface(lunarSurfaceObject, debugDeltaTime * speed * glm::normalize(direction));
    roverObject.front = glm::normalize(direction);
    roverObject.updateAndGetMainModelMatrixOnSurface(lunarSurfaceObject);
  }

  // function responsible for updating the scene and camera given user inputs and outputing the optical image to the window
  // PURELY FOR DEBUGGING / DEVELOPMENT USAGE
  // unoptimal way of rendering, but given that it is for debugging, I prefer this over cleaning up my code
  // not really meant to be used in the same executable as the other rendering functions due to a single shared camera
  // usage of this function in combination with getImageDataPackage() or getDepthDataPackage() will result in a flickering window
  // =============================================
  // bool showDVG ( = true ): bool defining if the DVG visualization needs to be rendered
  void debugRenderImage(bool showDVG = true) {
    if(glfwWindowShouldClose(window)) { return; }

    // input
    processInput(window);

    // transformation matrices
    view = camera.GetViewMatrix();
    // debug camera has custom planes so it can always see (nearly) the entire environment
    projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.001f, 25.f);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // DVG
    // =======================
    if(showDVG) {
      pointShader.use();
      pointShader.setMat4("projection", projection);
      pointShader.setMat4("view", view);

      for (size_t i = 0; i < DVGObjects.size(); i++)
      {
        pointShader.setVec3("pointColor", DVGObjects[i].color);
        DVGObjects[i].draw(pointShader);
      }
    }

    // Lines
    // =======================
    lineShader.use();
    lineShader.setMat4("projection", projection);
    lineShader.setMat4("view", view);
    for (size_t i = 0; i < linesObjects.size(); i++)
    {
     lineShader.setVec3("pointColor", linesObjects[i].color);
     linesObjects[i].draw(lineShader);
    }
    

    opticalShader.use();

    // lighting
    opticalShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    opticalShader.setVec3("lightDir", lightDir);

    // update transformation matrices
    opticalShader.setMat4("projection", projection);
    opticalShader.setMat4("view", view);

    // update and draw grid
    lunarSurfaceObject.draw(opticalShader);

    // update and draw rover
    roverObject.draw(opticalShader);

    // update and draw rest objects
    for (size_t i = 0; i < restObjects.size(); i++)
    {
      model = restObjects[i].meshes[0].getModelMatrix();
      opticalShader.setMat4("model", model);
      restObjects[i].draw(opticalShader);
    }
   
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // function responsible for initializing the Dynamic Voxel Grid Object with a given set of points
  // =============================================
  // std::vector<glm::vec3> points : vector of vec3 points defining the world positions of the center of the voxels in the Dynamic Voxel Grid
  // unsigned int index : index of DVGObject to set with given points in DVGObjects vector
  // glm::vec3 color : color of the rendered points in the DVGMesh indexed by "index"
  void setDVG(std::vector<glm::vec3>& points, unsigned int index, glm::vec3 color = glm::vec3{1.f, 0.f, 0.f}) {
    if(DVGObjects.size() < index) {
      std::cout << "Invalid index given in LZEngine::setDVG() : " << index << "\n";
      return;
    } else if (DVGObjects.size() == index){
      DVGObjects.push_back(DVGMesh{});
    }
    DVGObjects[index] = DVGMesh{};
    DVGObjects[index].updateDVG(points);
    DVGObjects[index].color = color;
  }

private:

  // pipelining and rendering buffers
  float* depthImage;

  float* opticalImageR;
  float* opticalImageG;
  float* opticalImageB;

  unsigned int FBO;
  unsigned int depthTexture;
  unsigned int depthDepthTexture;

  // buffer to calculate absolute paths
  char path[256];

  // exported file name count
  unsigned int pointcloudFileIndex = 0;
  unsigned int imageFileIndex = 0;

  // base rendering data initialization
  glm::vec3 lightDir = glm::normalize(glm::vec3{1.5f, -1.0f, 0.f});
  glm::mat4 model{1.f};
  glm::mat4 view{1.f};
  glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, NEAR_PLANE, FAR_PLANE);

  // function responsible for handling keyboard inputs
  // Only meant to be called somewhere in the debug rendering function callstack
  // =============================================
  // GLFWwindow* window : pointer to the active window instance
  void processInput(GLFWwindow *window)
  {
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
      camera.ProcessKeyboard(FORWARD, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
      camera.ProcessKeyboard(BACKWARD, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
      camera.ProcessKeyboard(LEFT, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
      camera.ProcessKeyboard(RIGHT, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
      camera.ProcessKeyboard(DOWN, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
      camera.ProcessKeyboard(UP, debugDeltaTime);

    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
      camera.ProcessKeyboard(TURNUP, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
      camera.ProcessKeyboard(TURNDOWN, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
      camera.ProcessKeyboard(TURNLEFT, debugDeltaTime);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
      camera.ProcessKeyboard(TURNRIGHT, debugDeltaTime);
  }
};

#endif