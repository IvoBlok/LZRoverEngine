#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "../slam/dynamicVoxelGrid.h"

#include <vector>
#include <map>
#include <list>

#define CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD 0.8f
#define CAN_BECOME_SEED_CURVATURE_THRESSHOLD 0.2f

class ClusterIDManager {
private:
  unsigned int lastID = 0;
  std::list<std::vector<long>> clusters;

public:
  unsigned int getUniqueClusterID() {
    clusters.push_back(std::vector<long>{});
    lastID++;
    return lastID;
  }

  void setVoxelClusterID(DVG& dvg, Voxel* voxel, unsigned int newClusterID) {
    dvg.getVoxelFromIndex(voxel->index)->clusterID = newClusterID;
    auto front = clusters.begin();
    std::advance(front, newClusterID - 1);

    front->push_back(voxel->index);
  }

  void linkClusters(DVG& dvg, unsigned int clusterIDToRemove, unsigned int clusterIDToKeep) {
    auto front1 = clusters.begin();
    std::advance(front1, clusterIDToRemove - 1);
    for (size_t i = 0; i < front1->size(); i++)
    {
      auto front2 = clusters.begin();
      std::advance(front2, clusterIDToKeep - 1);

      dvg.getVoxelFromIndex((*front1)[i])->clusterID = clusterIDToKeep;
      front2->push_back((*front1)[i]);
    }
    clusters.erase(front1);
  }
};


bool canVoxelGrowToVoxel(Voxel& voxel1, Voxel& voxel2) {
  return glm::dot(glm::normalize(voxel1.normal), glm::normalize(voxel2.normal)) < CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD;
}

bool canBeSeed(Voxel& voxel) {
  return true;
}

void growFromSeed(Voxel* seed, unsigned int newClusterID,  DVG& dvg, ClusterIDManager& idManager) {
  std::map<long, Voxel*> seeds;
  seeds[seed->index] = seed;

  while(seeds.size() != 0) {
    // get first, and remove it
    seed = seeds.begin()->second;
    seeds.erase(seeds.begin()->first);

    std::vector<Voxel> neighbours = dvg.getNeighbours(seed->centroid);
    neighbours.push_back(*seed);
    for (Voxel& voxel : neighbours) 
    {
      if(canVoxelGrowToVoxel(*seed, voxel)) {
        if(voxel.clusterID == 0) {
          // if the voxel we're looking at does not yet belong to a cluster, we set it to the new cluster ID
          idManager.setVoxelClusterID(dvg, &voxel, newClusterID);
          //if(canBeSeed(voxel)) {
          //  seeds[voxel.index] = &voxel;
          //}
        } else {
          // if the voxel we're looking at DOES already belong to a cluster, we merge the clusters
          idManager.linkClusters(dvg, voxel.clusterID, newClusterID);
        }
      }
    }
    
  }
}

void growClusters(DVG& dvg, ClusterIDManager& idManager, std::vector<long> newActiveVoxelIndices) {
  std::vector<Voxel*> initialseeds;
  for (size_t i = 0; i < newActiveVoxelIndices.size(); i++)
  {
    Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]);
    if(canBeSeed(*voxel))
      initialseeds.push_back(voxel);
  }

  for (size_t i = 0; i < initialseeds.size(); i++)
  {
    if(initialseeds[i]->clusterID == 0) {
      growFromSeed(initialseeds[i], idManager.getUniqueClusterID(), dvg, idManager);
    }
  }
}

#endif