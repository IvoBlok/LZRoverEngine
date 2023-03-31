#ifndef KDTREE_H
#define KDTREE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "voxel.h"
#include <glm/glm.hpp>

//#include "dynamicVoxelGrid.h"

// for the implementation of the KDTree here, the following page has pretty much been straight up copied
// https://rosettacode.org/wiki/K-d_tree
// adjustments are made to specify it towards 3 dimensions, which are stored for every point in a glm::vec3

struct leaf {
  leaf(Voxel* vx) : voxel(vx) {}

  Voxel* voxel;
  leaf* left;
  leaf* right;    
};

struct leaf_compare {
  leaf_compare(size_t index_) : index(index_) {}

  bool operator()(const leaf& leaf1, const leaf& leaf2) const {
    return leaf1.voxel->centroid[index] < leaf2.voxel->centroid[index];
  }

  size_t index;
};

class KDTree {
private:

  leaf* rootLeaf;
  leaf* bestLeaf;
  float bestDistance = 0;
  size_t visited = 0;
  std::vector<leaf> leafs;

  leaf* makeTree(size_t begin, size_t end, size_t index) {
    if (end <= begin)
      return nullptr;
    
    size_t n = begin + (end - begin)/2;
    auto i = leafs.begin();
    std::nth_element(i + begin, i + n, i + end, leaf_compare(index));
    index = (index + 1) % 3;
    leafs[n].left = makeTree(begin, n, index);
    leafs[n].right = makeTree(n + 1, end, index);
    return &leafs[n];
  }

  void nearest(leaf* root, const glm::vec3& point, size_t index) {
    if(root == nullptr)
      return;
    
    visited++;
    float distance = glm::dot(glm::vec3{root->voxel->centroid - point}, glm::vec3{root->voxel->centroid - point});
    if(bestLeaf == nullptr || distance < bestDistance) {
      bestDistance = distance;
      bestLeaf = root;
    }
    if(bestDistance == 0)
      return;
    
    float dx = root->voxel->centroid[index] - point[index];
    index = (index + 1) % 3;
    nearest(dx > 0 ? root->left : root->right, point, index);
    if(dx * dx >= bestDistance)
      return;
    nearest(dx > 0 ? root->right : root->left, point, index);
  }

public:
  KDTree(const KDTree&) = delete;
  KDTree& operator=(const KDTree&) = delete;

  KDTree() {}

  KDTree(std::vector<Voxel>& voxels) {
    for (size_t i = 0; i < voxels.size(); i++)
      leafs.push_back(leaf(&voxels[i]));
    rootLeaf = makeTree(0, leafs.size(), 0);
  }

  bool empty() const { return leafs.empty(); }

  size_t getVisited() const { return visited; }

  float distance() const { return std::sqrt(bestDistance); }

  const Voxel* nearest(const glm::vec3& pt) {
    if (rootLeaf == nullptr)
      throw std::logic_error("KDTree is empty");
    bestLeaf = nullptr;
    visited = 0;
    bestDistance = 0;
    nearest(rootLeaf, pt, 0);
    return bestLeaf->voxel;
  }
};

#endif