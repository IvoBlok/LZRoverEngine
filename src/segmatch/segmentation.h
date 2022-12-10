#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "../slam/dynamicVoxelGrid.h"
#include "normalEstimator.h"
#include "../lunarEmulation/LZEngine.h"

#include <vector>
#include <map>
#include <list>

#define CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD 12.f
#define MAX_ALLOWED_SEED_CURVATURE 0.15f

// ===============================
// ===============================
// Version 2
// ===============================
// ===============================

struct Cluster {
  int ID;
  std::vector<long> voxelIndices;

  Cluster(int id) {
    ID = id;
  }
};
std::list<Cluster> clusters;
std::list<Cluster> clustersWithoutGroundplane;

int clusterIDCounter = 0;
int clusterIDCounterGroundplane = 0;

// check if a given voxel can be a seed to start growing a cluster from
bool canBeSeed(DVG& dvg, NormalEstimator& normalEstimator,  Voxel* voxel) {
  // check if curvature is below some limit
  float curvature = normalEstimator.calculateNormalWithCovarianceMatrix(dvg, voxel->index);
  if(curvature)
    return curvature < (float)MAX_ALLOWED_SEED_CURVATURE;
  else 
    return false;
}

// check if two adjacent voxels follow the criteria for being in the same cluster
bool canGrowFromVoxelToVoxel(Voxel& voxel1, Voxel& voxel2) {
  return std::acos(glm::abs(glm::dot(glm::normalize(voxel1.normal), glm::normalize(voxel2.normal)))) * (360.f / 6.28f) < CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD;
}

// check if a given voxel is already part of a cluster
bool isPartOfCluster(Voxel& voxel) {
  return voxel.clusterID != 0;
}

bool isNotPartOfCluster(Voxel& voxel, int clusterID) {
  return voxel.clusterID != clusterID;
}

bool getClusterByID(std::list<Cluster>& clusters, int clusterID, std::reverse_iterator<std::list<Cluster>::iterator>& cluster, int* indexInClusters ) {
  int index = clusters.size() - 1;
  for (auto element = clusters.rbegin(); element != clusters.rend(); element++) {
    if(element->ID == clusterID) {
      cluster = element;
      if(indexInClusters != nullptr) { *indexInClusters = index; }
      return true;
    }
    index--;
  }
  return false;
}

// copy over all elements of one cluster to a given other, and remove the copied cluster from the list
// TODO potentially make this function itself choose which of the given clusters should be removed, based on the ID, and thus its age
bool mergeClusters(DVG& dvg, std::list<Cluster>& clusters, int clusterIDOfClusterToStay, int clusterIDToMergeAndRemove) {
  if(clusterIDOfClusterToStay == clusterIDToMergeAndRemove) 
    return false;

  // find the pointers to the clusters in question
  std::reverse_iterator<std::list<Cluster>::iterator> clusterToCopyOver;
  std::reverse_iterator<std::list<Cluster>::iterator> clusterToCopyTo;

  int indexOfClusterToRemove = 0;

  if(!getClusterByID(clusters, clusterIDToMergeAndRemove, clusterToCopyOver, &indexOfClusterToRemove)) { return false; }
  if(!getClusterByID(clusters, clusterIDOfClusterToStay, clusterToCopyTo, nullptr)) { return false; }

  // copy over the data and update the clusterID of the voxels in the cluster that is soon to be removed
  for (size_t i = 0; i < clusterToCopyOver->voxelIndices.size(); i++)
  {
    clusterToCopyTo->voxelIndices.push_back(clusterToCopyOver->voxelIndices[i]);
    dvg.getVoxelFromIndex(clusterToCopyOver->voxelIndices[i])->clusterID = clusterToCopyTo->ID;
  }
  
  // remove the copied cluster
  std::list<Cluster>::iterator iterator = clusters.begin();
  std::advance(iterator, indexOfClusterToRemove);
  clusters.erase(iterator);
  return true;
}

// update a voxels cluster state, updating both places where the cluster voxel relation is stored
//! does not check if the given voxel is already in the given cluster, given that the only implementation it is used doesn't require it
void setClusterID(Voxel* voxel, std::list<Cluster>& clusters, int clusterID) {
  // update the global list of clusters with a new entry
  std::reverse_iterator<std::list<Cluster>::iterator> cluster;
  if(!getClusterByID(clusters, clusterID, cluster, nullptr)) {
    std::cout << "ERROR: Invalid clusterID given!\n";
    return;
  }
  cluster->voxelIndices.push_back(voxel->index);

  // update the clusterID of the voxel (supposedly) in the DVG
  voxel->clusterID = clusterID;
}


void growFromSeed(DVG& dvg, NormalEstimator& normalEstimator, Voxel* voxel) {
  // check if the initial seed is already in a cluster
  if(voxel->clusterID != 0)
    return;

  // initialize seeds list and new cluster
  std::list<Voxel*> seeds = {voxel};
  
  // get a new unique cluster ID
  clusterIDCounter++;

  clusters.push_back(Cluster{clusterIDCounter});

  while(seeds.size() > 0) {
    Voxel* seed = seeds.front(); seeds.pop_front();
    setClusterID(seed, clusters, clusterIDCounter);

    std::vector<Voxel*> neighbours = dvg.getNeighbours(dvg.getIndexFromPoint(seed->centroid), 3);

    for (int j = 0; j < neighbours.size(); j++) {
      if(!isNotPartOfCluster(*neighbours[j], clusterIDCounter)) 
        continue;

      // check if the seed can grow to the neighbour
      if(canGrowFromVoxelToVoxel(*seed, *neighbours[j])) {
        if(isPartOfCluster(*neighbours[j])) {
          // if the neighbour to grow towards already is assigned a cluster, the clusters should be merged into one
          mergeClusters(dvg, clusters, clusterIDCounter, neighbours[j]->clusterID);
        } else {
          // if the neighbour to grow towards isn't part of a cluster, add it to cluster that the seed belongs to
          setClusterID(neighbours[j], clusters, clusterIDCounter);

          // check if the neighbour of the current seed can by itself be a seed, potentially further growing the cluster
          if(canBeSeed(dvg, normalEstimator, neighbours[j]))
            // a union check, as given in the paper this is based on, doesn't seem necessary given that we already know that it hasn't been part of cluster yet, 
            // and given that all seeds all automatically pushed to a cluster, all neighbours that pass the previous tests won't have been a seed before
            seeds.push_back(neighbours[j]); 
        }
      }
    }
  }
}

void growClusters(DVG& dvg, NormalEstimator& normalEstimator, std::vector<long> newActiveVoxelIndices) {
  // get seeds from newly active voxels
  std::vector<Voxel*> seeds;
  for (size_t i = 0; i < newActiveVoxelIndices.size(); i++) {
    Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]); 
    if(canBeSeed(dvg, normalEstimator, voxel))
      seeds.push_back(voxel);
  }
  
  // sort them by curvature
  // TODO

  // grow new clusters from every seed
  for (size_t i = 0; i < seeds.size(); i++)
  {
    growFromSeed(dvg, normalEstimator, seeds[i]);
  }
}




void growFromSeedWithoutGroundplane(DVG& dvg, NormalEstimator& normalEstimator, Voxel* voxel) {
  // check if the initial seed is already in a cluster
  if(voxel->clusterID != 0)
    return;

  // initialize seeds list and new cluster
  std::list<Voxel*> seeds = {voxel};
  
  // get a new unique cluster ID
  clusterIDCounterGroundplane--;

  clustersWithoutGroundplane.push_back(Cluster{clusterIDCounterGroundplane});

  while(seeds.size() > 0) {
    Voxel* seed = seeds.front(); seeds.pop_front();
    setClusterID(seed, clustersWithoutGroundplane, clusterIDCounterGroundplane);

    std::vector<Voxel*> neighbours = dvg.getNeighbours(dvg.getIndexFromPoint(seed->centroid), 2);
    
    for (int j = 0; j < neighbours.size(); j++) {
      std::cout << neighbours[j]->clusterID << " ";
      // filter out neighbours in ground plane
      //TODO Assuming here that the programs found the largest cluster to be the ground plane, which is very much not a given

      if(!isNotPartOfCluster(*neighbours[j], clusterIDCounterGroundplane)) 
        continue;

      // filter out ground plane neighbours
      if(neighbours[j]->clusterID > 0)
        continue;

      if(isPartOfCluster(*neighbours[j])) {
        // if the neighbour to grow towards already is assigned a cluster, the clusters should be merged into one
        mergeClusters(dvg, clustersWithoutGroundplane, clusterIDCounterGroundplane, neighbours[j]->clusterID);
      } else {
        // if the neighbour to grow towards isn't part of a cluster, add it to cluster that the seed belongs to
        setClusterID(neighbours[j], clustersWithoutGroundplane, clusterIDCounterGroundplane);
      }
    }
    std::cout << std::endl; 
  }
}

void growClustersWithoutGroundplane(DVG& dvg, NormalEstimator& normalEstimator, std::vector<long> newActiveVoxelIndices) {
  // get seeds from newly active voxels
  std::vector<Voxel*> seeds;
  for (size_t i = 0; i < newActiveVoxelIndices.size(); i++) {
    Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]); 
    if(voxel->clusterID == 0)
      seeds.push_back(voxel);
  }
  
  // sort them by curvature
  // TODO

  // grow new clusters from every seed
  for (size_t i = 0; i < seeds.size(); i++)
  {
    growFromSeedWithoutGroundplane(dvg, normalEstimator, seeds[i]);
  }
}

// horrible algorithm for visualization, scales terribly with time
void loadNormalsIntoEngine(DVG& dvg, LZEngine& engine) {
  int index = 0;
  for (auto cluster = clusters.rbegin(); cluster != clusters.rend(); cluster++) {
    // if a new cluster has been added since last time, make a new line rendering group
    if(index >= engine.linesObjects.size()) { 
      engine.linesObjects.push_back(LineMesh{}); 
      engine.linesObjects[index].color = glm::vec3{(float)rand()/(float)RAND_MAX, 0.f, 0.f};
    }

    // load the normals for the current cluster
    std::vector<glm::vec3> normals;
    for (size_t j = 0; j < cluster->voxelIndices.size(); j++)
    {
      Voxel* voxel = dvg.getVoxelFromIndex(cluster->voxelIndices[j]);
      normals.push_back(voxel->centroid);
      normals.push_back(voxel->normal);
    }
    
    engine.linesObjects[index].setLines(normals, 0.1f);
    index++;
  }

  for (auto cluster = clustersWithoutGroundplane.rbegin(); cluster != clustersWithoutGroundplane.rend(); cluster++) {
    // if a new cluster has been added since last time, make a new line rendering group
    if(index >= engine.linesObjects.size()) { 
      engine.linesObjects.push_back(LineMesh{}); 
      engine.linesObjects[index].color = glm::vec3{0.f, (float)rand()/(float)RAND_MAX, (float)rand()/(float)RAND_MAX};
    }

    // load the normals for the current cluster
    std::vector<glm::vec3> normals;
    for (size_t j = 0; j < cluster->voxelIndices.size(); j++)
    {
      Voxel* voxel = dvg.getVoxelFromIndex(cluster->voxelIndices[j]);
      normals.push_back(voxel->centroid);
      normals.push_back(voxel->normal);
    }
    
    engine.linesObjects[index].setLines(normals, 0.1f);
    index++;
  }

  // pop those rendering groups that are left over from previous iterations due to cluster merges in the current iteration
  for (size_t i = index; i < engine.linesObjects.size(); i++)
  {
    engine.linesObjects.pop_back();
  }
}

void displayClusterSizes() {
  for (auto cluster = clusters.rbegin(); cluster != clusters.rend(); cluster++) {
    std::cout << cluster->voxelIndices.size() << " : " << cluster->ID << std::endl;
  }
  std::cout << std::endl;
}

#endif