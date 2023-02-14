#include "lunarEmulation/LZEngine.h"
#include "segmatch/normalEstimator.h"
#include "segmatch/segmentation.h"
#include "pathplanning/pathplanning.h"
#include "slam/roverPoseEstimate.h"
#include "slam/dynamicVoxelGrid.h"
#include "slam/ICP.h"
#include "settings.h"

// Engine responsible for sensor emulation and debug window
LZEngine engine{POINTCLOUD_SCAN_WIDTH, POINTCLOUD_SCAN_HEIGHT};

// Class defining a simple system of enabling the correction of the error buildup of the positioning sensor on the rover
RoverPoseEstimate roverPoseEstimate;

// sensor data package definitions
RoverDepthDataPackage depthData;
// visualization data blocks
std::vector<glm::vec3> activeVoxels;
// buffer to store all voxels that became active recently and haven't successfully completed segmentation yet
std::vector<long> recentlyActivatedVoxelIndices;

// segmap classes initialization
DVG localMap;
GlobalMap globalMap;

void planPathAndMoveRover() {
  // DEBUG
  // boring path 'planning' solution to validate the rest of the solutions. This just moves the rover in a big circle
  // this should be replaced at some point by the path planning algorithm
  engine.updateDeltaTime();
  engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * lastFrame), 0.f, std::sin(0.1f * lastFrame)}, 0.025f);

  // filter largest groundplane for the safe movement area
  // std::vector<glm::vec3> safeArea = pathplanning::getSafeMovementVoxels(globalMap);

  // 2D project groundplane clusters into mesh / image / smth
  // .....

  // Check if a navigation update is necessary, and if yes, calculate the new path
  // .....
}

void applyDepthDataRegistration() {
  // retrieve new data from the rover sensors
  engine.updateIMUEstimate(APPLY_SYNTHETIC_NOISE_TO_EMULATED_SENSORS);
  engine.getDepthDataPackage(depthData);

  // update the position the software thinks the rover is at based on the IMU measurement that was sent together with the depth data package
  roverPoseEstimate.processNewIMUEstimate(depthData.pose);

  // transform depth measurements to estimated world space with pose estimate directly from pose measurement sensor
  for (size_t i = 0; i < depthData.pointclouds.size(); i++)
  {
    for (size_t j = 0; j < depthData.pointclouds[i].size(); j++)
    {
      // offset for the sensor position
      depthData.pointclouds[i][j] += engine.roverObject.depthCameraViewMatrices[i].relativePosition;

      // transform from relative to absolute purely for debug/visualization reasons
      depthData.pointclouds[i][j] = engine.getInitialRealPose() * glm::vec4{depthData.pointclouds[i][j], 1.f};

      // transform from rover space to initial pose space, where with 'initial pose space' the 3D space with principal axes aligned with those of the initial pose of the rover
      depthData.pointclouds[i][j] = depthData.pose * glm::vec4{depthData.pointclouds[i][j], 1.f};
    }
  }

  // Calculate the transformation from the new pointcloud with the estimated pose applied, to the current local map with partial ICP. 
  // The resulting matrix is the result of noise in mainly the Pose measuring device. ICP inherently also doesn't get to the exact answer, but it should be enough for this application
  glm::mat4 transformationEstimate = slam::getTransformationEstimateBetweenPointclouds(depthData.pointclouds, localMap);
  
  // update where the software thinks the rover is at, thus improving the accuracy of the localization
  roverPoseEstimate.modifyCurrentPoseEstimate(transformationEstimate);

  // Apply the newly found transformation matrix, and push the new pointcloud to the local DVG
  for (size_t i = 0; i < depthData.pointclouds.size(); i++){
    for (size_t j = 0; j < depthData.pointclouds[i].size(); j++) {
      depthData.pointclouds[i][j] = transformationEstimate * glm::vec4{depthData.pointclouds[i][j], 1.f};
    }
  }
  
  // Update the position the software thinks the rover is at with this new estimate
  // .....
  // TODO
}

void insertLocalMapToGlobalMap() {
  std::cout << "global map insertion occured!\n";
  
  // update globalmap with new entry
  globalMap.DVGs.push_back(localMap);

  // Calculate segment correspondences
  // .....

  // Update the global clusters in the global map from segment correspondences
  // .....
  //! PLACEHOLDER CODE
  /*
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
  */
  // Throw new relations into ISAM2, and retrieve the set of new transformation matrices
  // .....

  // Update the voxels in the globalDVGs with the new transformation matrices
  // .....

  // Update the transformation matrices stored in the globalDVGs, or something, not sure yet why they are also stored in the DVGs
  // .....

  // Reset the local map/DVG
  localMap.empty();
  recentlyActivatedVoxelIndices.clear();

  // Reset the data to visualize the clusters in the emulation engine, so that only the clusters in the local map are shown
  engine.linesObjects.empty();
}

void visualizeDataInEngine() {
  // export LOCAL active voxels to render engine
  //localDVG.getVoxelsForVisualization(activeVoxels);
  //engine.setDVG(activeVoxels, 0);
  
  // export GLOBAL active voxels to render engine
  //activeVoxels.clear();
  //for (size_t i = 0; i < globalDVGs.size(); i++)
  //  globalDVGs[i].getVoxelsForVisualization(activeVoxels, false, false);
  //engine.setDVG(activeVoxels, 1, glm::vec3{1.f, 1.f, 0.f});
  
  // export LOCAL normals to render engine
  Segmentation::loadClusterNormalsIntoEngine(localMap, engine);
}

int main(int argc, char const *argv[])
{
  engine.startEngine();

  int frame = 1;
  while (true)
  {    
    planPathAndMoveRover();

    if (frame % FRAMES_PER_SCAN == 0)
    {
      applyDepthDataRegistration();

      // Update Local DVG and update the buffer layers of the new active voxels
      localMap.insertPoints(depthData.pointclouds, recentlyActivatedVoxelIndices);

      // Incremental Normal Estimation
      std::vector<long> failedVoxels;
      NormalEstimator::calculateNormalsWithCovarianceMatrix(localMap, recentlyActivatedVoxelIndices, failedVoxels);

      // Incremental Segmentation
      Segmentation::growGroundplaneClusters(localMap, recentlyActivatedVoxelIndices);
      Segmentation::growObstacleClusters(localMap, recentlyActivatedVoxelIndices);

      // Incremental Feature Extraction
      // .....

      // Get the failed voxels into the to be processed voxels list of next iteration
      recentlyActivatedVoxelIndices.clear();
      recentlyActivatedVoxelIndices = failedVoxels;

      if(frame % (FRAMES_PER_SCAN * SCANS_PER_LOCAL_MAP) == 0) {
        insertLocalMapToGlobalMap();
      }
      
      visualizeDataInEngine(); 
    }

    do {
      engine.debugRenderImage();
    }
    while(START_NEXT_ITERATION_WITH_KEYPRESS_ACTIVE && glfwGetKey(engine.window, START_NEXT_ITERATION_WITH_KEYPRESS_KEY) != GLFW_PRESS);    

    frame++;
  }
  engine.stopEngine();

  return 1;
}