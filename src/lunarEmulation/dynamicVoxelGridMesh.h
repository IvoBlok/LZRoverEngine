#ifndef DYNAMIC_VOXEL_GRID_MESH_H
#define DYNAMIC_VOXEL_GRID_MESH_H

#include "mesh.h"

// class defining custom functions for the visualization object of a DVG
// given that it is purely for visualization, this is not optimized for speed
// The standard mesh structure is used, technically wasting memory on the GPU given that the DVG is for now only drawing points. 
// =============================================
class DVGMesh : public Mesh {
public:
  glm::vec3 color = glm::vec3{rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX};

  // default constructor
  DVGMesh() {};

  DVGMesh(std::vector<glm::vec3>& points, glm::vec3 color_ = glm::vec3{1.f, 0.2f, 0.2f}) : color(color_) {
    updateDVG(points);
  }

  // function responsible for initializing the vertices with the given points
  // =============================================
  // std::vector<glm::vec3> points : vector of vec3 points defining the world positions of the center of the voxels in the Dynamic Voxel Grid
  void initDVG(std::vector<glm::vec3>& points) {
    if(points.size() == 0) { return; }
    if(vertices.size() > 0) { std::cout << "Invalid initDVG call: DVGMesh is already initialized \n"; return; }
    for (size_t i = 0; i < points.size(); i++)
    {
      Vertex vertex{};
      vertex.Position = points[i];
      vertices.push_back(vertex);
    }
    
    setupMesh();
  }

  void updateDVG(std::vector<glm::vec3>& points) {
    if(vertices.size() == 0) { initDVG(points); return; }
    if(vertices.size() != points.size()) { std::cout << " DVGMesh update called with invalid source pointcloud size. Boem! >:) \n"; return; }
    for (size_t i = 0; i < points.size(); i++)
    {
      vertices[i].Position = points[i];
    }
  }

  // heavily simplified version of the parent class given that the DVG only renders points, has no textures and does not contribute to a potential framebuffer
  // =============================================
  // Shader& shader : shader program to be used to render this mesh
  void draw(Shader& shader)
  {
    shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices.size()));
    glBindVertexArray(0);
  }

  
  ~DVGMesh() {
    glBindVertexArray(VAO);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindVertexArray(0);
    
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
  }

private:

};

class LineMesh : public Mesh {
public:
  glm::vec3 color = glm::vec3{rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX};

private:
  bool needsMeshUpdate = true;

public:
  // default constructor
  LineMesh() {};

  void insertLine(glm::vec3 worldPos1, glm::vec3 worldPos2) {
    Vertex vertex1{};
    vertex1.Position = worldPos1;
    Vertex vertex2{};
    vertex2.Position = worldPos2;

    vertices.push_back(vertex1);
    vertices.push_back(vertex2);

    needsMeshUpdate = true;
  }

  void insertLine(glm::vec3 position, glm::vec3 direction, float length) {
    insertLine(position, position + length * glm::normalize(direction));
  }

  void insertLines(std::vector<glm::vec3>& normals, float length) {
    for (size_t i = 0; i < normals.size(); i+=2)
    {
      insertLine(normals[i], normals[i + 1], length);
    }
  }

  void setLines(std::vector<glm::vec3>& normals, float length) {
    vertices.clear();
    insertLines(normals, length);
  }

  // heavily simplified version of the parent class given that the DVG only renders points, has no textures and does not contribute to a potential framebuffer
  // =============================================
  // Shader& shader : shader program to be used to render this mesh
  void draw(Shader& shader)
  {
    if(needsMeshUpdate) { setupMesh(); needsMeshUpdate = false; }
    shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
    glBindVertexArray(0);
  }

  ~LineMesh() {
    glBindVertexArray(VAO);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindVertexArray(0);
    
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
  }
};
#endif