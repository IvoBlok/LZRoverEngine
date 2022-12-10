#ifndef NORMAL_ESTIMATOR_H
#define NORMAL_ESTIMATOR_H

#include "../slam/dynamicVoxelGrid.h"

#include <glm/glm.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Core>
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
  using Scalar = typename Eigen::Matrix3f::Scalar;

  glm::mat3 calculateCovarianceMatrix(std::vector<Voxel*>& voxels) {
    glm::mat3 covarMatrix = glm::mat3{0.f};
    glm::vec3 centroid = glm::vec3{0.f};

    for (size_t i = 0; i < voxels.size(); i++)
    {
      centroid += voxels[i]->centroid;
    }
    
    centroid.x /= voxels.size();
    centroid.y /= voxels.size();
    centroid.z /= voxels.size();

    unsigned int voxelCount = voxels.size();

    for (const auto& voxel : voxels)
    {
      glm::vec3 pt;
      pt = voxel->centroid - centroid;

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

  void computeRootsFromQuadratic(const Scalar& b, const Scalar& c, Eigen::Vector3f& roots) {
    // standard quadratic equation solution implementation
    roots(0) = Scalar(0);
    Scalar d = Scalar(b*b - 4.0*c);
    // check if terms are real, which in most usecases will be the case, given the characteriscs of symmetric square matrices
    if(d < 0.0)
      d = 0.0;

    Scalar sd = std::sqrt(d);

    roots(2) = 0.5f * (b + sd);
    roots(1) = 0.5f * (b - sd);
  }

  void computeRoots(const Eigen::Matrix3f& matrix, Eigen::Vector3f& roots) {
    Scalar c0 = matrix(0,0) * matrix(1,1) * matrix(2,2)
                + Scalar(2) * matrix(0,1) * matrix(0,2) * matrix(1,2)
                - matrix(0,0) * matrix(1,2) * matrix(1,2)
                - matrix(1,1) * matrix(0,2) * matrix(0,2)
                - matrix(2,2) * matrix(0,1) * matrix(0,1);
    
    Scalar c1 = matrix(0,0) * matrix(1,1)
                - matrix(0,1) * matrix(0,1)
                + matrix(0,0) * matrix(2,2)
                - matrix(0,2) * matrix(0,2)
                + matrix(1,1) * matrix(2,2)
                - matrix(1,2) * matrix(1,2);

    // c2 is effectively the trace of matrix
    Scalar c2 = matrix(0,0) + matrix(1,1) + matrix(2,2);

    if(std::abs(c0) < Eigen::NumTraits < Scalar > ::epsilon()) // one root is larger then 0 -> characteristic equation is quadratic
      computeRootsFromQuadratic(c2, c1, roots);
    else {
      Eigen::EigenSolver<Eigen::Matrix3f> es(matrix, false);
      Eigen::Vector3cf eigenvalues = es.eigenvalues();

      Eigen::Vector3f roots;
      roots(0) = eigenvalues(0).real();
      roots(1) = eigenvalues(1).real();
      roots(2) = eigenvalues(2).real();

      // Sort in increasing order
      if(roots(0) >= roots(1))
        std::swap(roots(0), roots(1));
      if(roots(1) >= roots(2)){
        std::swap(roots(1), roots(2));
        if(roots(0) >= roots(1)) 
          std::swap(roots(0), roots(1));
      }
      
      if(roots(0) <= 0) // eigenval for symmetric positive semi-definite matrix can not be negative! Set it to 0
        computeRootsFromQuadratic(c2, c1, roots);
    }
  }

  Eigen::Vector3f getLargest3x3EigenVector(const Eigen::Matrix3f& matrix) {
    using Index = typename Eigen::Matrix3f::Index;

    Eigen::Matrix3f crossProduct;
    crossProduct << matrix.row(0).cross(matrix.row(1)),
                    matrix.row(0).cross(matrix.row(2)),
                    matrix.row(1).cross(matrix.row(2));
    
    const auto len = crossProduct.rowwise().norm();

    Index index;
    const Scalar length = len.maxCoeff(&index);
    return crossProduct.row(index) / length;
  }

  float calculateNormalWithCovarianceMatrix(DVG& dvg, long index) {
    // get the voxel data for which the normal is being calcualted
    Voxel* voxel;
    voxel = dvg.getVoxelFromIndex(index);

    // get the direct grid neighbours of the voxel, excluding the voxel itself
    std::vector<Voxel*> neighbours = dvg.getNeighbours(index, 3);

    // add a copy of the main voxel to the list/vector
    Voxel v{};
    neighbours.push_back(&v);
    neighbours.back()->centroid = voxel->centroid;
    
    // calculate the covariance matrix of the neighbourhood
    // the elements of this matrix give the relation between two points in the neighboorhood; If one point gets further away, is the other more likely to also get further away, or the opposity, closer by?
    glm::mat3 covarMatrix = calculateCovarianceMatrix(neighbours);
    
    Eigen::Matrix3f matrix;
    matrix << covarMatrix[0][0], covarMatrix[1][0], covarMatrix[2][0], covarMatrix[0][1], covarMatrix[1][1], covarMatrix[2][1], covarMatrix[0][2], covarMatrix[1][2], covarMatrix[2][2];

    Scalar scale = matrix.cwiseAbs().maxCoeff();
    if(scale <= std::numeric_limits < Scalar > ::min())
      scale = Scalar(1.0);
    
    Eigen::Matrix3f scaledMatrix = matrix / scale;

    //Eigen::Vector3f eigenvalues;
    //computeRoots(scaledMatrix, eigenvalues);

    Eigen::Vector3f eigenvector = getLargest3x3EigenVector(scaledMatrix);

    // extract normal
    voxel->normal = glm::normalize(glm::vec3{eigenvector[0], eigenvector[1], eigenvector[2]});
    if(voxel->normal.y < 0) 
      voxel->normal *= -1.f;

    // calculate curvature
    float curvature = 0.f;
    neighbours.pop_back();
    for (size_t i = 0; i < neighbours.size(); i++)
    {
      //curvature += std::abs();
      glm::vec3 relativeVector = neighbours[i]->centroid - voxel->centroid;
      float sizeOfRelativeVec = std::pow(glm::length(relativeVector), 2);
      curvature += std::abs(glm::dot(relativeVector, voxel->normal)) * 1.f/sizeOfRelativeVec;
    }
    if(neighbours.size() == 0) {
      curvature = NULL;
    } else {
      curvature /= (float)neighbours.size();
    }
    return curvature;
    /*
    Scalar eig_sum = eigenvalues(0) + eigenvalues(1) + eigenvalues(2);//matrix.coeff(0) + matrix.coeff(4) + matrix.coeff(8);
    std::cout << eigenvalues(0) << " : " << eig_sum << std::endl;
    if(eig_sum != 0)
      return std::abs(eigenvalues(0) / eig_sum);
    else 
      return 0;
    */
  }

  void calculateNormalsWithCovarianceMatrix(DVG& dvg, std::vector<long>& voxelIndices) {
    for (size_t i = 0; i < voxelIndices.size(); i++)
    {
      calculateNormalWithCovarianceMatrix(dvg, voxelIndices[i]);
    }
  }
};

#endif