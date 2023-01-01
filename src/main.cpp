//#define EXPORTPOINTCLOUD
//#define EXPORTIMAGE
#include "lunarEmulation/LZEngine.h"
#include "slam/dynamicVoxelGrid.h"
#include "segmatch/normalEstimator.h"
#include "segmatch/segmentation.h"
#include "pathplanning/pathplanning.h"

int main(int argc, char const *argv[])
{

  // Engine responsible for sensor emulation and debug window
  LZEngine engine{160, 120};

  // sensor data package definitions
  RoverDepthDataPackage depthData;

  // visualization data blocks
  std::vector<glm::vec3> activeVoxels;

  // local map data blocks
  std::vector<long> recentlyActivatedVoxelIndices;

  // segmap classes initialization
  DVG localDVG;
  GlobalMap globalMap;

  // startup
  engine.startEngine();

  int i = 0;
  while (true)
  {    
    // move the rover in a given direction and let the engine physics do their thing
    engine.updateDeltaTime();
    engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * glfwGetTime()), 0.f, std::sin(0.1f * glfwGetTime())}, 0.1f);

    // Sensor data retrieval and processing
    if (i % 8 == 0)
    {
      // every given amount of measurements, copy the local map into the global map, and empty the local map for the next batch of measurements
      if(i % (8 * 25) == 0) {
        // update globalmap with new entry
        globalMap.DVGs.push_back(localDVG);

        // Calculate segment correspondences
        // .....

        // Update the global clusters in the global map from segment correspondences
        // .....
        //! PLACEHOLDER CODE
        Cluster* largestGroundplaneCluster = localDVG.getLargestGroundplaneCluster();

        if(globalMap.globalClusters.matchingClusterSets.size() == 0)
          globalMap.globalClusters.matchingClusterSets.push_back(MatchingClusterSet{});
        globalMap.globalClusters.matchingClusterSets.front().push_back(globalMap.DVGs.size(), *largestGroundplaneCluster);

        auto groundplaneCluster = localDVG.groundplaneClusters.rbegin();
        for (size_t i = 0; i < localDVG.groundplaneClusters.size(); i++)
        {
          if(&(*groundplaneCluster) == largestGroundplaneCluster)
            continue;
          groundplaneCluster++;
          MatchingClusterSet clusterSet{};
          clusterSet.push_back(globalMap.DVGs.size(), *groundplaneCluster);
          globalMap.globalClusters.matchingClusterSets.push_back(clusterSet);
        }
        
        // Throw new relations into ISAM2, and retrieve the set of new transformation matrices
        // .....

        // Update the voxels in the globalDVGs with the new transformation matrices
        // .....

        // Update the transformation matrices stored in the globalDVGs, or something, not sure yet why they are also stored in the DVGs
        // .....

        // Reset the local map/DVG
        localDVG.empty();
        recentlyActivatedVoxelIndices.clear();
      }

      // Calculate new IMU based rover location
      engine.updateIMUEstimate(false);

      // get depth + pose data
      engine.getDepthDataPackage(depthData);

      /* #region  Depth data registration */
      glm::mat3 inversePoseRot = glm::inverse(depthData.pose.rotation);
      glm::mat3 inverseInitPoseRot = glm::inverse(engine.getInitialRealPose().rotation);
      glm::vec3 initPoseTranslation = engine.getInitialRealPose().translation;

      // transform depth measurements to estimated world space with pose estimate directly from pose measurement sensor
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

      // Calculate the transformation from the new pointcloud with the estimated pose applied, to the current local map with partial ICP. 
      // The resulting matrix is the result of noise in mainly the Pose measuring device. ICP inherently also doesn't get to the exact answer, but it should be enough for this application
      // .....
      
      // Apply the newly found transformation matrix, and push the new pointcloud to the local DVG
      // .....

      /* #endregion */

      // Update Local DVG and update the buffer layers of the new active voxels
      localDVG.insertPoints(depthData.pointclouds, recentlyActivatedVoxelIndices);

      // Incremental Normal Estimation
      std::vector<long> failedVoxels;
      NormalEstimator::calculateNormalsWithCovarianceMatrix(localDVG, recentlyActivatedVoxelIndices, failedVoxels);

      // Incremental Segmentation
      Segmentation::growGroundplaneClusters(localDVG, recentlyActivatedVoxelIndices);
      Segmentation::growObstacleClusters(localDVG, recentlyActivatedVoxelIndices);

      // Incremental Feature Extraction
      // .....

      // Get the failed voxels into the to be processed voxels list of next iteration
      recentlyActivatedVoxelIndices.clear();
      recentlyActivatedVoxelIndices = failedVoxels;

      // Visualization
      // ============================
      // export LOCAL active voxels to render engine
      //localDVG.getVoxelsForVisualization(activeVoxels);
      //engine.setDVG(activeVoxels, 0);
      
      // export GLOBAL active voxels to render engine
      //activeVoxels.clear();
      //for (size_t i = 0; i < globalDVGs.size(); i++)
      //  globalDVGs[i].getVoxelsForVisualization(activeVoxels, false, false);
      //engine.setDVG(activeVoxels, 1, glm::vec3{1.f, 1.f, 0.f});
      
      // export LOCAL normals to render engine
      Segmentation::loadClusterNormalsIntoEngine(localDVG, engine);
    }

    // path planning 
    if (i % 8 * 4 == 0) {
      // filter largest groundplane for the safe movement area
      std::vector<glm::vec3> safeArea = pathplanning::getSafeMovementVoxels(globalMap);

      // 2D project groundplane clusters into mesh / image / smth
      // .....

      // Check if a navigation update is necessary, and if yes, calculate the new path
      // .....
    }

    // render for debugging camera
    engine.debugRenderImage();
    i++;
  }
  engine.stopEngine();

  return 1;
}