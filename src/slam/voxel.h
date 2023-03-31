#ifndef VOXEL_H
#define VOXEL_H

#include <glm/glm.hpp>

struct Voxel {
public:
  unsigned short int pointCount;
  long index;

  bool active = false;

  glm::vec3 centroid;
  glm::vec3 normal = glm::vec3{0.f};

  // 'clusterID' is set to 0 as the default, signalling that it is not part of any cluster/segment (yet).
  // 'clusterID' is negative non-zero if it's part of a groundplane cluster, and positive non-zero if it's part of a obstacle cluster
  // 'clusterID' can correspond to the 'temporary' cluster identifier, or the 'permanent' segment identifier. 
  short int clusterID = 0;

  Voxel(unsigned int count_, glm::vec3 centroid_, long index_, glm::vec3 normal_) : pointCount(count_), centroid(centroid_), index(index_), normal(normal_) {}

  Voxel() {}
};

#endif