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
  std::vector<glm::vec3> activeVoxels;
  std::vector<glm::vec3> newLocalNormals;
  std::vector<glm::vec3> newLocalNormals2;

  // local map data blocks
  std::vector<long> recentlyActivatedVoxelIndices[15];

  // segmap classes initialization
  DVG localDVG;
  DVG globalDVG;
  NormalEstimator incrNormalEstimator;

  // startup
  engine.startEngine();

  int i = 0;
  while (true)
  {    
    // move the rover
    engine.updateDeltaTime();
    engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * glfwGetTime()), 0.f, std::sin(0.1f * glfwGetTime())}, 0.1f);

    // take measurements every given amount of walking cycles
    if (i % 8 == 0)
    {
      // every given amount of measurements, copy the local map into the global map, and empty the local map for the next batch of measurements
      if(i % (8 * 25) == 0) {
        globalDVG.copyAndRemoveDVG(localDVG);

        for (size_t i = 0; i < 15; i++)
          recentlyActivatedVoxelIndices[i].clear();
      }

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
        recentlyActivatedVoxelIndices[i] = recentlyActivatedVoxelIndices[i + 1];
      
      recentlyActivatedVoxelIndices[14] = localDVG.insertPoints(depthData.pointclouds);

      // Incremental Normal Estimation
      incrNormalEstimator.calculateNormalsWithCovarianceMatrix(localDVG, recentlyActivatedVoxelIndices[10]);

      // Incremental Segmentation
      Segmentation::growGroundplaneClusters(localDVG, incrNormalEstimator, recentlyActivatedVoxelIndices[10]);
      Segmentation::growObstacleClusters(localDVG, incrNormalEstimator, recentlyActivatedVoxelIndices[0]);

      // Incremental Feature Extraction
      // ....

      // Visualization
      // ============================
      // export Local and Glboal DVG To render engine for debug camera
      localDVG.getVoxelsForVisualization(activeVoxels);
      engine.setDVG(activeVoxels, 0);
      globalDVG.getVoxelsForVisualization(activeVoxels);
      engine.setDVG(activeVoxels, 1, glm::vec3{1.f, 1.f, 0.f});
      
      // visualize local normals
      Segmentation::loadNormalsIntoEngine(localDVG, engine);
    }

    // render for debugging camera
    engine.debugRenderImage();
    i++;
  }
  engine.stopEngine();

  return 1;
}