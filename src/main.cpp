//#define EXPORTPOINTCLOUD
//#define EXPORTIMAGE
#include "lunarEmulation/LZEngine.h"
#include "slam/dynamicVoxelGrid.h"
#include "segmatch/normalEstimator.h"
#include "segmatch/segmentation.h"

int main(int argc, char const *argv[])
{


  // Engine responsible for sensor emulation and debug window
  LZEngine engine{160, 120};

  // startup
  engine.startEngine();
  engine.lunarSurfaceObject.baseSurfaceHighDetail.generateLunarSurface(200, 200, 0.05f, "../resources/textures/sandDiffuseMap.png", 0.004f, glm::vec2{100.f, 100.f});

  // sensor data package definitions
  RoverDepthDataPackage depthData;

  // visualization data blocks
  std::vector<glm::vec3> activeLocalVoxels;

  // segmap classes initialization
  DVG localDVG;
 

  // generate rocks
  std::vector<float> sizes = engine.lunarSurfaceObject.rocks.generateRockSizes(100);
  for (size_t x = 0; x < 10; x++)
  {
    for (size_t z = 0; z < 10; z++)
    {
      engine.lunarSurfaceObject.rocks.generateRock(glm::vec3{(float)x * 5.f + 1.0f, 0.f, (float)z * 5.f}, glm::vec3{sizes[x*10+z]}, 4);
      std::cout << "1 done!\n";
    }
  }

  for (size_t x = 0; x < 10; x++)
  {
    for (size_t z = 0; z < 10; z++)
    {
      engine.roverObject.setRoverPosition(glm::vec3{(float)x * 5.f, 0.f, (float)z * 5.f});
      std::cout << "2 done!\n";

      for (size_t p = 0; p < 15; p++) {    
        // get depth + pose data
        engine.getDepthDataPackage(depthData);

        /* #endregion */

        localDVG.insertPoints(depthData.pointclouds);

        // Visualization
        // ============================
        // export Local DVG To render engine for debug camera
        localDVG.getVoxelsForVisualization(activeLocalVoxels);
        engine.setDVG(activeLocalVoxels, 0);
      }

      localDVG.outputVoxelPointsToFile("rockscan" + std::to_string(x*10 + z) + ".txt");
      
      localDVG.voxels.clear();
    }
  }
  engine.stopEngine();

  return 1;
}