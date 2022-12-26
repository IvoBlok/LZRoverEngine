#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "../slam/dynamicVoxelGrid.h"
#include "normalEstimator.h"
#include "../lunarEmulation/LZEngine.h"

#include <vector>
#include <map>

#define CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD 12.f
#define MAX_ALLOWED_SEED_CURVATURE 0.18f
#define CLUSTERING_GROUNDPLANE_NEIGHBOUR_SEARCH_RADIUS 2
#define CLUSTERING_OBSTACLE_NEIGHBOUR_SEARCH_RADIUS 1
#define MIN_VOXEL_COUNT_TO_BECOME_SEGMENT 10000

namespace Segmentation {

  class ClusterIDGenerator {
  private:
    int clusterIDCounterObstacles = 0;
    int clusterIDCounterGroundplane = 0;

    int segmentIDCounterObstacles = 0;
    int segmentIDCounterGroundplane = 0;
    
  public:
    int getNewObstacleClusterID() { return ++clusterIDCounterObstacles; }
    int getNewGroundplaneClusterID() { return --clusterIDCounterGroundplane; }

    int getNewObstacleSegmentID() { return ++segmentIDCounterObstacles; }
    int getNewGroundplaneSegmentID() { return --segmentIDCounterGroundplane; }
  };
  ClusterIDGenerator clusterIDGen;

  // check if a given voxel can be a seed to start growing a cluster from
  bool canBeSeed(DVG& dvg, Voxel* voxel) {
    // check if curvature is below some limit
    float curvature;
    if(NormalEstimator::calculateNormalWithCovarianceMatrix(dvg, voxel->index, curvature)) {
      return curvature < (float)MAX_ALLOWED_SEED_CURVATURE;
    }
    return false;
  }

  // check if two adjacent voxels follow the criteria for being in the same cluster
  bool canGrowFromVoxelToVoxel(Voxel& voxel1, Voxel& voxel2) {
    return voxel1.normal != glm::vec3{0.f} && voxel2.normal != glm::vec3{0.f} &&
      std::acos(glm::abs(glm::dot(glm::normalize(voxel1.normal), glm::normalize(voxel2.normal)))) * (360.f / 6.28f) < CAN_GROW_TO_VOXEL_ANGLE_THRESHOLD;
  }

  // check if a given voxel is already part of a cluster
  bool isPartOfCluster(Voxel& voxel) {
    return voxel.clusterID != 0;
  }

  bool isNotPartOfCluster(Voxel& voxel, int clusterID) {
    return voxel.clusterID != clusterID;
  }

  bool getClusterByID(std::list<Cluster>& clusters, int clusterID, std::reverse_iterator<std::list<Cluster>::iterator>& cluster, int* indexInClusters, bool isSegment = false ) {
    int index = clusters.size() - 1;
    for (auto element = clusters.rbegin(); element != clusters.rend(); element++) {
      if(element->ID == clusterID && element->isSegment == isSegment) {
        cluster = element;
        if(indexInClusters != nullptr) { *indexInClusters = index; }
        return true;
      }
      index--;
    }
    return false;
  }

  void convertClusterToSegment(DVG& dvg, std::reverse_iterator<std::list<Cluster>::iterator> cluster) {
    cluster->isSegment = true;
    if(cluster->ID < 0) { cluster->ID = clusterIDGen.getNewGroundplaneSegmentID(); } 
    else { cluster->ID = clusterIDGen.getNewObstacleSegmentID(); }

    for (size_t i = 0; i < cluster->voxelIndices.size(); i++)
    {
      Voxel* voxel =dvg.getVoxelFromIndex(cluster->voxelIndices[i]);
      voxel->isPartOfSegment = true;
      voxel->clusterID = cluster->ID;
    }
  }

  // copy over all elements of one cluster to a given other, and remove the copied cluster from the list
  bool mergeClusters(DVG& dvg, std::list<Cluster>& clusters, int clusterIDOfClusterToStay, bool clusterToStayIsSegment, int clusterIDToMergeAndRemove, bool clusterToMergeAndRemoveIsSegment) {
    if(clusterIDOfClusterToStay == clusterIDToMergeAndRemove && clusterToStayIsSegment == clusterToMergeAndRemoveIsSegment) 
      return false;

    bool switchClusterId = (clusterToMergeAndRemoveIsSegment == true && clusterToStayIsSegment == false) 
                        || (clusterToMergeAndRemoveIsSegment == true && clusterToStayIsSegment == true && std::abs(clusterIDOfClusterToStay) < std::abs(clusterIDToMergeAndRemove)); 

    // find the pointers to the clusters in question
    std::reverse_iterator<std::list<Cluster>::iterator> clusterToCopyOver;
    std::reverse_iterator<std::list<Cluster>::iterator> clusterToCopyTo;

    int indexOfClusterToRemove = 0;

    if(!getClusterByID(clusters, clusterIDToMergeAndRemove, clusterToCopyOver, &indexOfClusterToRemove, clusterToMergeAndRemoveIsSegment)) { return false; }
    if(!getClusterByID(clusters, clusterIDOfClusterToStay, clusterToCopyTo, nullptr, clusterToStayIsSegment)) { return false; }

    // copy over the data and update the clusterID of the voxels in the cluster that is soon to be removed
    for (size_t i = 0; i < clusterToCopyOver->voxelIndices.size(); i++)
    {
      clusterToCopyTo->voxelIndices.push_back(clusterToCopyOver->voxelIndices[i]);
      dvg.getVoxelFromIndex(clusterToCopyOver->voxelIndices[i])->clusterID = clusterToCopyTo->ID;
    }

    if(switchClusterId)
      for (size_t i = 0; i < clusterToCopyOver->voxelIndices.size(); i++)
        dvg.getVoxelFromIndex(clusterToCopyOver->voxelIndices[i])->clusterID = clusterToCopyOver->ID;

    // check if the merge has pushed the cluster over the minimum voxel count to become a segment
    if(!clusterToCopyTo->isSegment && clusterToCopyTo->voxelIndices.size() >= MIN_VOXEL_COUNT_TO_BECOME_SEGMENT)
      convertClusterToSegment(dvg, clusterToCopyTo);

    // remove the copied cluster
    std::list<Cluster>::iterator iterator = clusters.begin();
    std::advance(iterator, indexOfClusterToRemove);
    clusters.erase(iterator);
    return true;
  }

  // update a voxels cluster state, updating both places where the cluster voxel relation is stored
  //! does not check if the given voxel is already in the given cluster, given that the only implementation it is used doesn't require it
  void setClusterID(DVG& dvg, Voxel* voxel, std::list<Cluster>& clusters, int clusterID, bool isSegment = false) {
    // update the global list of clusters with a new entry
    std::reverse_iterator<std::list<Cluster>::iterator> cluster;
    if(!getClusterByID(clusters, clusterID, cluster, nullptr, isSegment)) {
      std::cout << "ERROR: Invalid clusterID given!\n";
      return;
    }
    cluster->voxelIndices.push_back(voxel->index);
    
    // if this addition of a voxel to the cluster means that the cluster has reached the segment threshold, convert it to a segment
    if(cluster->voxelIndices.size() == MIN_VOXEL_COUNT_TO_BECOME_SEGMENT) {
      convertClusterToSegment(dvg, cluster);
    }
    // update the clusterID of the voxel (supposedly) in the DVG
    voxel->clusterID = clusterID;
  }

  void growGroundplaneFromSeed(DVG& dvg, Voxel* voxel) {
    // check if the initial seed is already in a cluster
    if(voxel->clusterID != 0)
      return;

    // get a new unique cluster ID
    int newGroundplaneClusterID = clusterIDGen.getNewGroundplaneClusterID();

    // initialize seeds list and new cluster
    std::list<Voxel*> seeds = {voxel};
    dvg.groundplaneClusters.push_back(Cluster{newGroundplaneClusterID});

    while(seeds.size() > 0) {
      Voxel* seed = seeds.front(); seeds.pop_front();
      setClusterID(dvg, seed, dvg.groundplaneClusters, newGroundplaneClusterID, voxel->isPartOfSegment);

      std::vector<Voxel*> neighbours = dvg.getNeighbours(dvg.getIndexFromPoint(seed->centroid), CLUSTERING_GROUNDPLANE_NEIGHBOUR_SEARCH_RADIUS);

      for (int j = 0; j < neighbours.size(); j++) {
        // check if the neighbour isn't already part of the new cluster
        if(!isNotPartOfCluster(*neighbours[j], newGroundplaneClusterID)) 
          continue;

        // check if the neighbour isn't already part of an obstacle cluster
        if(neighbours[j]->clusterID > 0)
          continue;

        // check if the seed can grow to the neighbour
        if(canGrowFromVoxelToVoxel(*seed, *neighbours[j])) {
          if(isPartOfCluster(*neighbours[j])) {
            // if the neighbour to grow towards already is assigned a cluster, the clusters should be merged into one
            mergeClusters(dvg, dvg.groundplaneClusters, newGroundplaneClusterID, voxel->isPartOfSegment, neighbours[j]->clusterID, neighbours[j]->isPartOfSegment);
          } else {
            // if the neighbour to grow towards isn't part of a cluster, add it to cluster that the seed belongs to
            setClusterID(dvg, neighbours[j], dvg.groundplaneClusters, newGroundplaneClusterID, voxel->isPartOfSegment);

            // check if the neighbour of the current seed can by itself be a seed, potentially further growing the cluster
            if(canBeSeed(dvg, neighbours[j]))
              // a union check, as given in the paper this is based on, doesn't seem necessary given that we already know that it hasn't been part of cluster yet, 
              // and given that all seeds all automatically pushed to a cluster, all neighbours that pass the previous tests won't have been a seed before
              seeds.push_back(neighbours[j]); 
          }
        }
      }
    }
  }

  void growGroundplaneClusters(DVG& dvg, std::vector<long> newActiveVoxelIndices) {
    if(newActiveVoxelIndices.size() == 0) {
      return;
    }
    // get seeds from newly active voxels
    std::vector<Voxel*> seeds;
    for (size_t i = 0; i < newActiveVoxelIndices.size(); i++) {
      Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]); 
      if(canBeSeed(dvg, voxel))
        seeds.push_back(voxel);
    }
    
    // sort them by curvature
    // TODO

    // grow new clusters from every seed
    for (size_t i = 0; i < seeds.size(); i++)
    {
      growGroundplaneFromSeed(dvg, seeds[i]);
    }
  }

  void growObstaclesFromSeed(DVG& dvg, Voxel* voxel) {
    // check if the initial seed is already in a cluster
    if(voxel->clusterID != 0)
      return;

    // initialize seeds list and new cluster
    std::list<Voxel*> seeds = {voxel};
    
    // get a new unique cluster ID
    int newObstacleClusterID = clusterIDGen.getNewObstacleClusterID();

    dvg.obstacleClusters.push_back(Cluster{newObstacleClusterID});

    while(seeds.size() > 0) {
      Voxel* seed = seeds.front(); seeds.pop_front();
      setClusterID(dvg, seed, dvg.obstacleClusters, newObstacleClusterID, voxel->isPartOfSegment);

      std::vector<Voxel*> neighbours = dvg.getNeighbours(dvg.getIndexFromPoint(seed->centroid), CLUSTERING_OBSTACLE_NEIGHBOUR_SEARCH_RADIUS);
      
      for (int j = 0; j < neighbours.size(); j++) {
        // filter out neighbours that are already part of the new cluster
        if(!isNotPartOfCluster(*neighbours[j], newObstacleClusterID)) 
          continue;

        // filter out ground plane neighbours
        if(neighbours[j]->clusterID < 0)
          continue;

        if(isPartOfCluster(*neighbours[j])) {
          // if the neighbour to grow towards already is assigned a cluster, the clusters should be merged into one
          mergeClusters(dvg, dvg.obstacleClusters, newObstacleClusterID, voxel->isPartOfSegment, neighbours[j]->clusterID, neighbours[j]->isPartOfSegment);
        } else {
          // if the neighbour to grow towards isn't part of a cluster, add it to cluster that the seed belongs to
          setClusterID(dvg, neighbours[j], dvg.obstacleClusters, newObstacleClusterID, voxel->isPartOfSegment);
        }
      }
    }
  }

  void growObstacleClusters(DVG& dvg, std::vector<long> newActiveVoxelIndices) {
    if(newActiveVoxelIndices.size() == 0) {
      return;
    }
    // get seeds from newly active voxels that haven't been clustered into a groundplane cluster yet
    std::vector<Voxel*> seeds;
    for (size_t i = 0; i < newActiveVoxelIndices.size(); i++) {
      Voxel* voxel = dvg.getVoxelFromIndex(newActiveVoxelIndices[i]); 
      if(voxel->clusterID == 0)
        seeds.push_back(voxel);
    }
    
    // grow new clusters from every seed
    for (size_t i = 0; i < seeds.size(); i++)
    {
      growObstaclesFromSeed(dvg, seeds[i]);
    }
  }

  // debug/visualization functions
  // ============================================
  void loadClusterNormalsIntoEngine(std::vector<DVG>& dvgs, LZEngine& engine) {
    // horrible algorithm for visualization, scales terribly with time
    int index = 0;
    for (auto dvg = dvgs.begin(); dvg != dvgs.end(); dvg++) {
      for (auto cluster = dvg->groundplaneClusters.rbegin(); cluster != dvg->groundplaneClusters.rend(); cluster++) {
        // if a new cluster has been added since last time, make a new line rendering group
        if(index >= engine.linesObjects.size()) { 
          engine.linesObjects.push_back(LineMesh{}); 
          engine.linesObjects[index].color = glm::vec3{(float)rand()/(float)RAND_MAX, 0.f, 0.f};
        }

        // load the normals for the current cluster
        std::vector<glm::vec3> normals;
        for (size_t j = 0; j < cluster->voxelIndices.size(); j++)
        {
          Voxel* voxel = dvg->getVoxelFromIndex(cluster->voxelIndices[j]);
          normals.push_back(voxel->centroid);
          normals.push_back(voxel->normal);
        }
        
        engine.linesObjects[index].setLines(normals, 0.1f);
        index++;
      }

      for (auto cluster = dvg->obstacleClusters.rbegin(); cluster != dvg->obstacleClusters.rend(); cluster++) {
        // if a new cluster has been added since last time, make a new line rendering group
        if(index >= engine.linesObjects.size()) { 
          engine.linesObjects.push_back(LineMesh{}); 
          engine.linesObjects[index].color = glm::vec3{0.f, (float)rand()/(float)RAND_MAX, (float)rand()/(float)RAND_MAX};
        }

        // load the normals for the current cluster
        std::vector<glm::vec3> normals;
        for (size_t j = 0; j < cluster->voxelIndices.size(); j++)
        {
          Voxel* voxel = dvg->getVoxelFromIndex(cluster->voxelIndices[j]);
          normals.push_back(voxel->centroid);
          normals.push_back(voxel->normal);
        }
        
        engine.linesObjects[index].setLines(normals, 0.1f);
        index++;
      }
    }


    // pop those rendering groups that are left over from previous iterations due to cluster merges in the current iteration
    for (size_t i = index; i < engine.linesObjects.size(); i++)
    {
      engine.linesObjects.pop_back();
    }
  }

  void loadClusterNormalsIntoEngine(DVG& dvg, LZEngine& engine) {
    std::vector<DVG> dvgs;
    dvgs.push_back(dvg);
    loadClusterNormalsIntoEngine(dvgs, engine);
  }

  void displayClusterSizes(DVG& dvg) {
    for (auto cluster = dvg.obstacleClusters.rbegin(); cluster != dvg.obstacleClusters.rend(); cluster++) {
      std::cout << cluster->voxelIndices.size() << " : " << cluster->ID << std::endl;
    }
    std::cout << std::endl;
  }
}
#endif