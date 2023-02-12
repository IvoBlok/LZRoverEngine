//#define EXPORTPOINTCLOUD
//#define EXPORTIMAGE
#include "lunarEmulation/LZEngine.h"
#include "slam/dynamicVoxelGrid.h"
#include "segmatch/normalEstimator.h"
#include "segmatch/segmentation.h"
#include "pathplanning/pathplanning.h"
#include "slam/ICP.h"
#include "settings.h"

// Engine responsible for sensor emulation and debug window
LZEngine engine{POINTCLOUD_SCAN_WIDTH, POINTCLOUD_SCAN_HEIGHT};

// sensor data package definitions
RoverDepthDataPackage depthData;
// visualization data blocks
std::vector<glm::vec3> activeVoxels;
// local map data blocks
std::vector<long> recentlyActivatedVoxelIndices;

// segmap classes initialization
DVG localMap;
GlobalMap globalMap;

void planPathAndMoveRover() {
  // DEBUG
  // boring path 'planning' solution to validate the rest of the solutions. This just moves the rover in a big circle
  // this should be replaced at some point by the path planning algorithm
  engine.updateDeltaTime();
  engine.moveRoverInDirection(glm::vec3{std::cos(0.1f * glfwGetTime()), 0.f, std::sin(0.1f * glfwGetTime())}, 0.1f);

  // filter largest groundplane for the safe movement area
  // std::vector<glm::vec3> safeArea = pathplanning::getSafeMovementVoxels(globalMap);

  // 2D project groundplane clusters into mesh / image / smth
  // .....

  // Check if a navigation update is necessary, and if yes, calculate the new path
  // .....
}

void applyDepthDataRegistration() {
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
  glm::mat4 transformationEstimate = slam::getTransformationEstimateBetweenPointclouds(depthData.pointclouds, localMap);
  
  // Apply the newly found transformation matrix, and push the new pointcloud to the local DVG
  for (size_t i = 0; i < depthData.pointclouds.size(); i++){
    for (size_t j = 0; j < depthData.pointclouds[i].size(); j++) {
      depthData.pointclouds[i][j] = transformationEstimate * glm::vec4{depthData.pointclouds[i][j], 1.f};
    }
  }
  
  // DEBUG
  std::cout << "=-=-=-=-=-=-=-=-=-=-=\n";
  std::cout << transformationEstimate[0][0] << " " << transformationEstimate[0][1] << " " << transformationEstimate[0][2] << " " << transformationEstimate[0][3] << std::endl;
  std::cout << transformationEstimate[1][0] << " " << transformationEstimate[1][1] << " " << transformationEstimate[1][2] << " " << transformationEstimate[1][3] << std::endl;
  std::cout << transformationEstimate[2][0] << " " << transformationEstimate[2][1] << " " << transformationEstimate[2][2] << " " << transformationEstimate[2][3] << std::endl;

  // Update the position the software thinks the rover is at with this new estimate
  // .....
  // TODO
}

void insertLocalMapToGlobalMap() {
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
  // startup
  engine.startEngine();

  int frame = 0;
  while (true)
  {    
    planPathAndMoveRover();

    // Sensor data retrieval and processing
    // ============================
    if (frame % FRAMES_PER_SCAN == 0)
    {
      engine.updateIMUEstimate(false);
      engine.getDepthDataPackage(depthData);

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

    engine.debugRenderImage();
    frame++;
  }
  engine.stopEngine();

  return 1;
}