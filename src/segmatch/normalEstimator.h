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
      const Scalar s_inv3 = Scalar(1.0/3.0);
      const Scalar s_sqrt3 = std::sqrt(Scalar(3.0));

      Scalar c2_over_3 = c2 * s_inv3;
      Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
      if(a_over_3 > Scalar(0))
        a_over_3 = Scalar(0);
      
      Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));

      Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
      if(q > Scalar(0))
        q = Scalar(0);

      // Compute the eigenvalues by so;lving for the roots of the fancy polynomial
      Scalar rho = std::sqrt(-a_over_3);
      Scalar theta = std::atan2(std::sqrt(-q), half_b) * s_inv3;
      Scalar cos_theta = std::cos(theta);
      Scalar sin_theta = std::sin(theta);
      roots(0) = c2_over_3 + Scalar(2) * rho * cos_theta;
      roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
      roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

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
    // the elements of this matrix give the relation between two points in the neighboorhood; If one point gets further away, is the other more likely to also get further away, or the opposity, closer by?
    glm::mat3 covarMatrix = calculateCovarianceMatrix(neighbours);
    
    Eigen::Matrix3f matrix;
    matrix << covarMatrix[0][0], covarMatrix[1][0], covarMatrix[2][0], covarMatrix[0][1], covarMatrix[1][1], covarMatrix[2][1], covarMatrix[0][2], covarMatrix[1][2], covarMatrix[2][2];

    Scalar scale = matrix.cwiseAbs().maxCoeff();
    if(scale <= std::numeric_limits < Scalar > ::min())
      scale = Scalar(1.0);
    
    Eigen::Matrix3f scaledMatrix = matrix / scale;

    Eigen::Vector3f eigenvalues;
    computeRoots(scaledMatrix, eigenvalues);

    Scalar eigenvalue = eigenvalues(0) * scale;
    scaledMatrix.diagonal().array() -= eigenvalues(0);
    Eigen::Vector3f eigenvector = getLargest3x3EigenVector(scaledMatrix);

    voxel->normal = glm::normalize(glm::vec3{eigenvector[0], eigenvector[1], eigenvector[2]});
    if(voxel->normal.y < 0) 
      voxel->normal *= -1.f;
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