#ifndef NORMAL_ESTIMATOR_H
#define NORMAL_ESTIMATOR_H

#include "../slam/dynamicVoxelGrid.h"

#include <glm/glm.hpp>
#include <Eigen/Eigenvalues>

#include <vector>


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// IN EARLY STAGES OF DEVELOPMENT
// IN EARLY STAGES OF DEVELOPMENT
// IN EARLY STAGES OF DEVELOPMENT
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Incremental normal estimation
class NormalEstimator {
public:

  float avgNeighbourCount = 0.f;
  std::vector<glm::vec3> normalsVis;


  glm::mat3 calculateCovarianceMatrix(std::vector<Voxel>& voxels) {
    glm::mat3 covarMatrix = glm::mat3{0.f};
    glm::vec3 centroid = glm::vec3{0.f};

    for (size_t i = 0; i < voxels.size(); i++)
    {
      centroid += voxels[i].centroid;
    }
    
    centroid.x /= voxels.size();
    centroid.y /= voxels.size();
    centroid.z /= voxels.size();

    unsigned int voxelCount = voxels.size();

    for (const auto& voxel : voxels)
    {
      glm::vec3 pt;
      pt = voxel.centroid - centroid;

      covarMatrix[0][0] +=pt.x * pt.x;
      covarMatrix[2][2] +=pt.z * pt.z;
      
      covarMatrix[1][2] +=pt.y * pt.z;
      covarMatrix[2][1] +=pt.y * pt.z;

      covarMatrix[0][1] +=pt.y * pt.x;
      covarMatrix[1][0] +=pt.y * pt.x;

      covarMatrix[0][2] +=pt.x * pt.z;
      covarMatrix[2][0] +=pt.x * pt.z;
    }

    return covarMatrix;
  }

  void calculateNormalWithCovarianceMatrix(DVG& dvg, long index) {
    // get the voxel data for which the normal is being calcualted
    Voxel* voxel;
    voxel = dvg.getVoxelFromIndex(index);

    // get the direct grid neighbours of the voxel, excluding the voxel itself
    std::vector<Voxel> neighbours = dvg.getNeighbours(index);

    // add a copy of the main voxel to the list/vector
    neighbours.push_back(Voxel{});
    neighbours.back().centroid = voxel->centroid;
    
    // calculate the covariance matrix of the neighbourhood
    // the elements of this matrix give the relation between two points in the neighboorhood. If one is gets further away, is the other more likely to get further away, or closer by
    glm::mat3 covarMatrix = calculateCovarianceMatrix(neighbours);

    // calculate eigenvectors and eigenvalues
    // ================================================
    // convert covariance matrix to Eigen format
    Eigen::Matrix<float, 3, 3> matrix;
    matrix << covarMatrix[0][0], covarMatrix[1][0], covarMatrix[2][0], covarMatrix[0][1], covarMatrix[1][1], covarMatrix[2][1], covarMatrix[0][2], covarMatrix[1][2], covarMatrix[2][2];


    Eigen::EigenSolver<Eigen::Matrix<float, 3, 3>> solver;
    solver.compute(matrix);
    /*
    //
    // solve for eigenvectors + eigenvalues. Covariance matrices are always symmetric and have all zeros on it's diagonal. 
    // This means that it always has only real eigenvalues, and thus we can skip the filtering for complex values
    Eigen::VectorXf eigenValues = solver.eigenvalues();
    Eigen::MatrixXf eigenVectors = solver.eigenvectors();
    std::vector<std::tuple<float, Eigen::Vector3f>> eigenVectorsAndValues;

    for (size_t i = 0; i < eigenValues.size(); i++)
    {
      std::tuple<float, Eigen::Vector3f> VecAndVal(eigenValues[i], eigenVectors[i]);
      eigenVectorsAndValues.push_back(VecAndVal);
    }

    // sort the eigenvectors bij eigenvalue
    std::sort(eigenVectorsAndValues.begin(), eigenVectorsAndValues.end(), 
    [&](const std::tuple<float, Eigen::Vector3f>& a, const std::tuple<float, Eigen::Vector3f>& b) -> bool {
      return std::get<0>(a) <= std::get<0>(b);
    });

    // eigenvector corresponding to the smallest eigenvalue is our estimated normal
    Eigen::Vector3f normal = std::get<1>(eigenVectorsAndValues.back());
    voxel->normal = glm::normalize(glm::vec3{normal.x(), normal.y(), normal.z()});
    */
    unsigned int smallestEigenValueIndex = 0;
    float smallestEigenValue = solver.eigenvalues()[0].real();

    for (unsigned int i = 1; i < solver.eigenvalues().size(); i++)
    {
      if(solver.eigenvalues()[i].real() < smallestEigenValue) {
        smallestEigenValueIndex = i;
        smallestEigenValue = solver.eigenvalues()[i].real();
      }
    }

    voxel->normal = glm::normalize(glm::vec3{solver.eigenvectors().row(smallestEigenValueIndex).x().real(), solver.eigenvectors().row(smallestEigenValueIndex).y().real(), solver.eigenvectors().row(smallestEigenValueIndex).z().real()});
   
    // debug
    if (voxel->normal.y < 0.f) {voxel->normal *= -1.f;}

    std::cout << matrix << std::endl << std::endl;
    std::cout << solver.eigenvectors() << std::endl;
    std::cout << solver.eigenvalues() << std::endl;
    std::cout << voxel->normal.x << " : " << voxel->normal.y << std::endl << std::endl;
  }

  void calculateNormal(DVG& dvg, long index, bool renderLines = false) {
    std::vector<Voxel> neighbours = dvg.getNeighbours(index);

    Voxel* voxel;
    voxel = dvg.getVoxelFromIndex(index);

    // update running avg
    avgNeighbourCount += neighbours.size();

    if(neighbours.size() <= 1) { return; }

    glm::vec3 sum;
    for (size_t i = 0; i < neighbours.size() - 1; i++)
    {
      glm::vec3 vec1 = neighbours[i].centroid - voxel->centroid;
      glm::vec3 vec2 = neighbours[i + 1].centroid - voxel->centroid;

      if(renderLines) {
        normalsVis.push_back(voxel->centroid);
        normalsVis.push_back(vec1);
      }

      sum += glm::normalize(glm::cross(vec1, vec2));
    }
    // normalize
    if(sum.y < 0) { sum *= -1; }

    voxel->normal = glm::normalize(sum);
  }

  void calculateNormalsWithCovarianceMatrix(DVG& dvg, std::vector<long>& voxelIndices) {
    for (size_t i = 0; i < voxelIndices.size(); i++)
    {
      calculateNormalWithCovarianceMatrix(dvg, voxelIndices[i]);
    }
  }

  void calculateNormals(DVG& dvg, std::vector<long>& voxelIndices) {
    avgNeighbourCount = 0.f;
    for (size_t i = 0; i < voxelIndices.size(); i++)
    {
      if(i == 0) {
        calculateNormal(dvg, voxelIndices[i], true);
      } else {
        calculateNormal(dvg, voxelIndices[i]);
      }
    }

    avgNeighbourCount = avgNeighbourCount / (float)voxelIndices.size();
    std::cout << "average neighbours: " << avgNeighbourCount << std::endl;
  }



private:
  


};

#endif