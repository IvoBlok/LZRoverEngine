//#define EXPORTPOINTCLOUD
//#define EXPORTIMAGE
#include "lunarEmulation/LZEngine.h"
#include "slam/dynamicVoxelGrid.h"
#include "segmatch/normalEstimator.h"
#include "segmatch/segmentation.h"

int main(int argc, char const *argv[])
{

  // Engine responsible for sensor emulation and debug window
  LZEngine engine{120, 90};

  // sensor data package definitions
  RoverDepthDataPackage depthData;
  RoverImageDataPackage imageData;

  // visualization data blocks
  std::vector<glm::vec3> activeLocalVoxels;
  std::vector<glm::vec3> newLocalNormals;
  std::vector<glm::vec3> newLocalNormals2;

  // local map data blocks
  std::vector<long> newActiveVoxelsIndices;
  std::vector<long> lastNewActiveVoxelsIndices;
  std::vector<long> lastNewActiveVoxelsIndices2;

  // segmap classes initialization
  DVG localDVG;
  NormalEstimator incrNormalEstimator;

  // startup
  engine.startEngine();

  // set debug line renderer colors for the different types
  engine.linesObjects.push_back(LineMesh{});
  engine.linesObjects.push_back(LineMesh{});
  engine.linesObjects.push_back(LineMesh{});
  engine.linesObjects.push_back(LineMesh{});
  engine.linesObjects[0].color = glm::vec3{0.f, 0.f, 1.f};
  engine.linesObjects[1].color = glm::vec3{0.f, 1.f, 1.f};
  engine.linesObjects[2].color = glm::vec3{1.f, 1.f, 0.f};
  engine.linesObjects[3].color = glm::vec3{1.f, 1.f, 1.f};

  int i = 0;
  while (true)
  {    
    // move the rover
    engine.updateDeltaTime();
    glm::vec3 lastRoverPos = engine.roverObject.getRoverPosition();
    engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * glfwGetTime()), 0.f, std::sin(0.1f * glfwGetTime())}, 0.1f);
    
    engine.linesObjects[0].insertLine(lastRoverPos, engine.roverObject.getRoverPosition());

    // take measurements every given amount of walking cycles
    if (i % 5 == 0)
    {
      glm::vec3 lastRoverIMUGuess = engine.initialRealPose.translation + engine.IMUPoseEstimate.translation;
      // Calculate new IMU based rover location
      engine.updateIMUEstimate(false);
      engine.linesObjects[1].insertLine(lastRoverIMUGuess, engine.initialRealPose.translation + engine.IMUPoseEstimate.translation);

      // get depth + pose data
      engine.getDepthDataPackage(depthData);

      // update total trajectory
      // ....

      /* #region  Depth data registration */
      glm::mat3 inversePoseRot = glm::inverse(depthData.pose.rotation);
      glm::mat3 inverseInitPoseRot = glm::inverse(engine.getInitialRealPose().rotation);
      glm::vec3 initPoseTranslation = engine.getInitialRealPose().translation;

      for (size_t i = 0; i < depthData.pointclouds.size(); i++)
      {
        for (size_t j = 0; j < depthData.pointclouds[i].size(); j++)
        {
          // offset for the sensor position
          depthData.pointclouds[i][j] += engine.roverObject.depthCameraViewMatrices[i].relativePosition;

          // transform to coords relative to initial rover state
          depthData.pointclouds[i][j] = inversePoseRot * depthData.pointclouds[i][j];

          // translate form initial pose position
          depthData.pointclouds[i][j] += depthData.pose.translation;

          // transform from relative to absolute purely for debug reasons
          depthData.pointclouds[i][j] += initPoseTranslation;
          depthData.pointclouds[i][j] = inverseInitPoseRot * depthData.pointclouds[i][j];
        }
      }
      /* #endregion */

      // Update Local DVG
      lastNewActiveVoxelsIndices2 = lastNewActiveVoxelsIndices;
      lastNewActiveVoxelsIndices = newActiveVoxelsIndices;
      newActiveVoxelsIndices = localDVG.insertPoints(depthData.pointclouds);

      // Incremental Normal Estimation
      incrNormalEstimator.calculateNormalsWithCovarianceMatrix(localDVG, lastNewActiveVoxelsIndices2);

      // Segmentation
      growClusters(localDVG, incrNormalEstimator, lastNewActiveVoxelsIndices2);

      // Ground plane removal
      // ....

      // Incremental Feature Extraction
      // ....

      // Visualization
      // ============================
      // export Local DVG To render engine for debug camera
      localDVG.getVoxelsForVisualization(activeLocalVoxels);
      engine.setDVG(activeLocalVoxels, 0);
      // normals
      localDVG.getNormalsForVisualization(newLocalNormals, true);
      engine.linesObjects[2].setLines(newLocalNormals, 0.1f);

      localDVG.getNonGroundPlaneNormalsForVisualization(newLocalNormals2);
      engine.linesObjects[3].setLines(newLocalNormals2, 0.06f);
    }

    // save to file
    localDVG.outputVoxelPointsToFile();

    // render for debugging camera
    engine.debugRenderImage();
    i++;
  }
  engine.stopEngine();

  return 1;
}