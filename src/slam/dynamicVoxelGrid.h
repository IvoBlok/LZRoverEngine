#ifndef DYNAMIC_VOXEL_GRID_H 
#define DYNAMIC_VOXEL_GRID_H

#include <glm/glm.hpp>

#include <vector>
#include <map>
#include <iterator>
#include <memory>
#include <list>
#include <utility>

#include <fstream>
#include <iostream>

#include "voxel.h"
#include "KDTree.h"
#include "../settings.h"

struct Cluster {
  int ID;
  bool isSegment = false;
  std::vector<long> voxelIndices;

  Cluster(int id) {
    ID = id;
  }
};

// this datablock contains references to all clusters in any of the DVGs of the global map that have been matched to the same singular obstacle.
struct MatchingClusterSet {
  std::vector<std::pair<size_t, Cluster&>> clusters;

  void push_back(size_t indexInGlobalMap, Cluster& cluster) {
    clusters.push_back(std::pair<size_t, Cluster&>{indexInGlobalMap, cluster});
  }           

  int getClusterSetSize() {
    int size = 0;
    for (size_t i = 0; i < clusters.size(); i++)
      size += clusters[i].second.voxelIndices.size();
    return size;
  }
};

// this datablock contains references to all clusters in any of the DVGs of the global map, sorted in groups so that every group corresponds to an obstacle.
// Every cluster reference in a set has been classified as part of that obstacle.
struct GlobalClusters {
  std::vector<MatchingClusterSet> matchingClusterSets;
};

class DVG {
public:
  // 'voxels' contains the actual data the DVG is holding. It's one big block in memory, with its elements sorted by the Voxel index parameter. 
  // This index is calculated from the voxel centroid with a 3D - 1D mapping. 
  // Because of the data structure used, a reference or pointer to an element might be invalidated if new elements are inserted in the vector.
  std::vector<Voxel> voxels;
  // two lists of clusters contain the voxels that have been clustered together by the segmentation algorithm. 
  // one has negative clusterID's, one positive, to differentiate them easier from a voxel.
  std::list<Cluster> groundplaneClusters;
  std::list<Cluster> obstacleClusters;

  glm::mat4 transformationMatrix;

  // can store the points in the DVG in a tree-like structure, making the process of finding closest points significantly faster
  slam::KDTree* tree = nullptr;

  // max 63, so the max value of a long is unique, and reserved for signaling non-initialized
  unsigned int lengthBitCount = 12;
  unsigned int widthBitCount = 12;
  unsigned int heightBitCount = 8;

  // scaling factor of the coordinates. conversion to voxel side length in world space = 1.f / (float)resolution
  float resolution = RESOLUTION_DVG;
  // minimum amount of points required in a voxel to make it active
  unsigned int pointsRequiredForActiveVoxel = REQUIRED_POINTCOUNT_TO_BECOME_ACTIVE;

  std::pair<bool, unsigned int> binarySearch(long index) {
    if (voxels.size() == 0 || voxels[0].index > index) {return {false, 0}; }

    unsigned int lower = 0;
    unsigned int upper = voxels.size() - 1;

    while (lower <= upper) {
      unsigned int middle = (lower + upper) / 2;
      if(middle >= voxels.size()) { return {false, upper}; }
      long indexAtMiddle = voxels[middle].index;

      if(index == indexAtMiddle) {
        return {true, middle};
      }

      if(index < indexAtMiddle) {
        upper = middle - 1;
      } else {
        lower = middle + 1;
      }
    }
    return {false, lower};
  }

  long getIndexFromPoint(glm::vec3& point) {
    // calculate index
    long index = (long)std::floor(resolution * point.x) 
                + ((long)std::floor(resolution * point.y) << lengthBitCount)
                + ((long)std::floor(resolution * point.z) << (lengthBitCount + widthBitCount));
    return index;
  }

  Voxel* getVoxelFromIndex(long index) {
    std::pair<bool, unsigned int> result = binarySearch(index);

    if(result.first) { return &voxels[result.second]; }

    return nullptr;
  }

  bool insertPoint(glm::vec3& point, long& indexToSet, unsigned int count = 1, glm::vec3 normal = glm::vec3{0.f}, int clusterID = 0) {
    
    long index = getIndexFromPoint(point);

    // calculate place in vector
    std::pair<bool, unsigned int> vectorLocation = binarySearch(index);
    
    // update the vector accordingly
    if(vectorLocation.first) {

      Voxel& voxel = voxels[vectorLocation.second];
      // update the pointCount to reflect the added point
      voxel.pointCount += count;
      // update the centroid (i.e. the average of the point positions within the voxel) with the new point position
      voxel.centroid = ((float)(voxel.pointCount - 1) * voxel.centroid + point) / (float)voxel.pointCount;
      
      if(voxel.pointCount - count < pointsRequiredForActiveVoxel && voxel.pointCount >= pointsRequiredForActiveVoxel) { voxel.active = true; indexToSet = index; return true; }

    } else {
      
      // make new voxel, update it's data and push insert it in the correct location
      Voxel voxel;
      voxel.pointCount = count;
      voxel.centroid = point;
      voxel.index = index;
      voxel.clusterID = clusterID;
      voxel.normal = normal;
      voxels.insert(voxels.begin() + vectorLocation.second, voxel);

      if(pointsRequiredForActiveVoxel <= count) { voxels[vectorLocation.second].active = true; indexToSet = index; return true; }
    }

    return false;
  }

  bool insertPoint(glm::vec3& point, unsigned int count = 1, glm::vec3 normal = glm::vec3{0.f}, int clusterID = 0) {
    long placeholderIndex;
    return insertPoint(point, placeholderIndex, count, normal, clusterID);
  }

  std::vector<long> insertPoints(std::vector<glm::vec3> points) {
    std::vector<long> newlyActiveVoxels;
    
    long index;
    for (size_t i = 0; i < points.size(); i++)
    {
      if(insertPoint(points[i], index)) { newlyActiveVoxels.push_back(index); }
    }

    return newlyActiveVoxels;
  }

  std::vector<long> insertPoints(std::vector<std::vector<glm::vec3>> pointclouds) {
    std::vector<long> newlyActiveVoxels;
    for (size_t i = 0; i < pointclouds.size(); i++)
    {
      std::vector<long> newVoxels = insertPoints(pointclouds[i]);
      newlyActiveVoxels.insert(newlyActiveVoxels.end(), newVoxels.begin(), newVoxels.end());
    }
    return newlyActiveVoxels;
  }

  void insertPoints(std::vector<std::vector<glm::vec3>> pointclouds, std::vector<long>& newlyActiveVoxelList) {
    for (size_t i = 0; i < pointclouds.size(); i++)
    {
      std::vector<long> newVoxels = insertPoints(pointclouds[i]);
      newlyActiveVoxelList.insert(newlyActiveVoxelList.end(), newVoxels.begin(), newVoxels.end());
    }
    return;
  }

  void insertPoints(DVG& dvg) {
    for (size_t i = 0; i < dvg.voxels.size(); i++)
    {
      insertPoint(dvg.voxels[i].centroid, dvg.voxels[i].pointCount, dvg.voxels[i].normal, dvg.voxels[i].clusterID);
    }
  }

  void insertClusters(DVG& dvg) {
    for (auto cluster = dvg.groundplaneClusters.rbegin(); cluster != dvg.groundplaneClusters.rend(); cluster++)
    {
      groundplaneClusters.push_back(*cluster);
    }
    for (auto cluster = dvg.obstacleClusters.rbegin(); cluster != dvg.obstacleClusters.rend(); cluster++)
    {
      obstacleClusters.push_back(*cluster);
    }
  }

  void copyAndRemoveDVG(DVG& dvg) {
    insertClusters(dvg);
    insertPoints(dvg);

    dvg.empty();
  }

  void empty() {
    obstacleClusters.clear();
    groundplaneClusters.clear();
    voxels.clear();
  }

  Cluster* getLargestGroundplaneCluster() {
    Cluster* largestCluster;
    for(auto& cluster : groundplaneClusters) {
      if(!largestCluster || cluster.voxelIndices.size() > largestCluster->voxelIndices.size())
        largestCluster = &cluster;
    }
    return largestCluster;
  }

  // slow variant, Voxel data is copied over and index is calculated in a far from optimal way
  std::vector<Voxel*> getNeighbours(long index, int range = 2) {
    std::vector<Voxel*> neighbours;

    glm::vec3 voxelPos = getVoxelFromIndex(index)->centroid;

    for (int x = -range; x <= range; x++)
    {
      for (int y = -range; y <= range; y++)
      {
        for (int z = -range; z <= range; z++)
        {
          if(x == 0 && y == 0 && z == 0) { continue; }

          glm::vec3 vec = glm::vec3{(float)x / resolution,(float)y / resolution, (float)z / resolution} + voxelPos;
          long index1 = getIndexFromPoint(vec);
          std::pair<bool, unsigned int> result = binarySearch(index1);
          if(result.first && voxels[result.second].active) { neighbours.push_back(&voxels[result.second]); }
        }
      }
    }
    return neighbours;
  }

  std::vector<Voxel*> getNeighbours(glm::vec3 position, int range = 2) {
    return getNeighbours(getIndexFromPoint(position), range);
  }

  // Debug functionality
  // ===========================
  void getVoxelsForVisualization(std::vector<glm::vec3>& visualizationPointList, bool returnAllVoxels = false, bool clearList = true) {
    if(clearList)
      visualizationPointList.clear();
    for (size_t i = 0; i < voxels.size(); i++)
    {
      if(voxels[i].active || returnAllVoxels) {
        visualizationPointList.push_back(voxels[i].centroid);
      }
    }
    return;
  }

  void getNormalsForVisualization(std::vector<glm::vec3>& normalsList, bool onlyGroundPlane = false, bool clearList = true) {
    if(clearList)
      normalsList.clear();
    for (size_t i = 0; i < voxels.size(); i++)
    {
      if(voxels[i].active  && (!onlyGroundPlane || voxels[i].clusterID == 1)) {
        normalsList.push_back(voxels[i].centroid);
        normalsList.push_back(voxels[i].normal);
      }
    }
  }

  void getClusterNormals(std::vector<glm::vec3>& normalsList, int ID, bool clearList = true) {
    if(clearList)
      normalsList.clear();
    for (size_t i = 0; i < voxels.size(); i++) {
      if(voxels[i].active && voxels[i].clusterID == ID) {
        normalsList.push_back(voxels[i].centroid);
        normalsList.push_back(voxels[i].normal);
      }
    }
  }

  void getNormalsForVisualization(std::vector<glm::vec3>& normalsList, std::vector<Voxel*> voxels_, bool clearList = true) {
    if(clearList)
      normalsList.clear();
    for (size_t i = 0; i < voxels_.size(); i++)
    {
      normalsList.push_back(voxels_[i]->centroid);
      normalsList.push_back(voxels_[i]->normal);
    }
  }

  void outputVoxelPointsToFile(std::string outputFileName = "voxelOutput.txt") {
    std::fstream outputFile;
    outputFile.open(outputFileName, ios::out);
    for (size_t i = 0; i < voxels.size(); i++)
    {
      outputFile << "(" << voxels[i].centroid.x << ", " << voxels[i].centroid.y << ", " << voxels[i].centroid.z
                  << "), ";
    }
  }
  
  void outputVoxelDatToFile(std::string outputFileName = "voxelOutput.txt") {
    std::fstream outputFile;
    outputFile.open(outputFileName, ios::out);
    for (size_t i = 0; i < voxels.size(); i++)
    {
      outputFile << "(" << voxels[i].centroid.x << ", " << voxels[i].centroid.y << ", " << voxels[i].centroid.z << ", "
                        << voxels[i].normal.x << ", " << voxels[i].normal.y << ", " << voxels[i].normal.z
                  << "), ";
    }
  }
};

class GlobalMap {
public:
  std::vector<DVG> DVGs;
  GlobalClusters globalClusters;

private:

};

#endif