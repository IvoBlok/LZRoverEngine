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
  std::vector<long> recentlyActivatedVoxelIndices[15];

  // segmap classes initialization
  DVG localDVG;
  NormalEstimator incrNormalEstimator;

  // startup
  engine.startEngine();

  // set debug line renderer colors for the different types

  int i = 0;
  while (true)
  {    
    // move the rover
    engine.updateDeltaTime();
    glm::vec3 lastRoverPos = engine.roverObject.getRoverPosition();
    engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * glfwGetTime()), 0.f, std::sin(0.1f * glfwGetTime())}, 0.1f);

    // take measurements every given amount of walking cycles
    if (i % 8 == 0)
    {
      glm::vec3 lastRoverIMUGuess = engine.initialRealPose.translation + engine.IMUPoseEstimate.translation;
      // Calculate new IMU based rover location
      engine.updateIMUEstimate(false);

      // get depth + pose data
      engine.getDepthDataPackage(depthData);

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

      // Update Local DVG and update the buffer layers of the new active voxels
      for (size_t i = 0; i < 15 - 1; i++)
      {
        recentlyActivatedVoxelIndices[i] = recentlyActivatedVoxelIndices[i + 1];
      }
      
      recentlyActivatedVoxelIndices[14] = localDVG.insertPoints(depthData.pointclouds);

      // Incremental Normal Estimation
      incrNormalEstimator.calculateNormalsWithCovarianceMatrix(localDVG, recentlyActivatedVoxelIndices[10]);

      // Segmentation
      growClusters(localDVG, incrNormalEstimator, recentlyActivatedVoxelIndices[10]);
      growClustersWithoutGroundplane(localDVG, incrNormalEstimator, recentlyActivatedVoxelIndices[0]);

      // Incremental Feature Extraction
      // ....

      // Visualization
      // ============================
      // export Local DVG To render engine for debug camera
      localDVG.getVoxelsForVisualization(activeLocalVoxels);
      engine.setDVG(activeLocalVoxels, 0);
      
      // visualize normals
      loadNormalsIntoEngine(localDVG, engine);
    }

    // render for debugging camera
    engine.debugRenderImage();
    i++;
  }
  engine.stopEngine();

  return 1;
}