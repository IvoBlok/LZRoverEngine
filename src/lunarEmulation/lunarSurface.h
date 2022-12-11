#ifndef LUNAR_SURFACE_H
#define LUNAR_SURFACE_H

#include "mesh.h"

#include <chrono>

class BaseLunarSurface : public Mesh {
public:
  unsigned int gridXSquareCount;
  unsigned int gridZSquareCount;
  glm::vec2 offset;
  float squareSize;

  // default constructor
  BaseLunarSurface() {}

  // Generates a Mesh consisting of a given amount of squares with y = 0 forming a plane with given square size
  void generateGridPlane(unsigned int xSquareCount, unsigned int zSquareCount, float squareLength, const char *texturePath, float textureScale, glm::vec2 translate)
  {
    // generate vertices
    for (int x = 0; x < xSquareCount + 1; x++)
    {
      for (int z = 0; z < zSquareCount + 1; z++)
      {
        vertices.push_back(Vertex{});
        vertices[vertices.size() - 1].Position = glm::vec3{(float)x * squareLength, 0, (float)z * squareLength} + glm::vec3{translate[0], 0, translate[1]};
        vertices[vertices.size() - 1].TexCoords = glm::vec2{(float)x / ((float)xSquareCount * textureScale), (float)z / ((float)zSquareCount * textureScale)};
      }
    }
    // generate indices
    for (int z = 0; z < zSquareCount; z++)
    {
      for (int x = 0; x < xSquareCount; x++)
      {
        // triangle 1
        indices.push_back(((xSquareCount + 1) * x + z));
        indices.push_back(((xSquareCount + 1) * x + z + 1));
        indices.push_back(((xSquareCount + 1) * (x + 1) + z));
        // triangle 2
        indices.push_back(((xSquareCount + 1) * (x + 1) + z));
        indices.push_back(((xSquareCount + 1) * x + z + 1));
        indices.push_back(((xSquareCount + 1) * (x + 1) + z + 1));
      }
    }

    gridXSquareCount = xSquareCount;
    gridZSquareCount = zSquareCount;
    offset = translate;
    squareSize = squareLength;

    // texture
    loadTexture(texturePath, "texture_diffuse");

    setupMesh();
    return;
  }

  // Retrieves the y-value of the closest vertex (projected onto the x,z plane) to a given x,z position
  float getHeightOnSmoothSurfaceWithGridPos(glm::vec3 gridPosition)
  {
    glm::vec2 XZGridPosition{gridPosition.x, gridPosition.z};

    float deltaX = gridPosition[0] - floor(gridPosition[0]);
    float deltaZ = gridPosition[2] - floor(gridPosition[2]);

    XZGridPosition[0] = floor(XZGridPosition[0]);
    XZGridPosition[1] = floor(XZGridPosition[1]);

    float height = vertices[(gridXSquareCount + 1) * (int)XZGridPosition[0] + (int)XZGridPosition[1]].Position.y;
    float height2 = vertices[(gridXSquareCount + 1) * ((int)XZGridPosition[0] + 1) + (int)XZGridPosition[1]].Position.y;
    float height3 = vertices[(gridXSquareCount + 1) * (int)XZGridPosition[0] + (int)XZGridPosition[1] + 1].Position.y;
    float height4 = vertices[(gridXSquareCount + 1) * ((int)XZGridPosition[0] + 1) + (int)XZGridPosition[1] + 1].Position.y;

    height += deltaX * (height2 - height);
    height3 += deltaX * (height4 - height3);
    height += deltaZ * (height3 - height);

    return height;
  }
  
  float getHeightOnSmoothSurfaceWithWorldPos(glm::vec3 worldPosition)
  {
    // check if it lies within the boundary of the grid
    glm::vec2 edgeVec1 = offset;
    glm::vec2 edgeVec2 = edgeVec2 + squareSize * glm::vec2{gridXSquareCount, gridZSquareCount};
    
    // if the given point is out of the XZ area covered by the grid, return NULL
    if(worldPosition.x <= edgeVec1.x || worldPosition.z <= edgeVec1.y || worldPosition.x >= edgeVec2.x || worldPosition.z >= edgeVec2.y) {
        return 0;
    }
    
    // convert it to grid-relative coordinates
    worldPosition.x /= squareSize;
    worldPosition.z /= squareSize;
    return getHeightOnSmoothSurfaceWithGridPos(worldPosition);
  }

  // Calculates the (normalized) normal vector of a given grid at a given 3d location
  glm::vec3 calculateNormalOnSmoothSurfaceWithGridPos(glm::vec3 gridPosition)
  {
    float currentHeight = getHeightOnSmoothSurfaceWithGridPos(gridPosition);

    glm::vec3 vec1{squareSize, getHeightOnSmoothSurfaceWithGridPos(gridPosition + glm::vec3{1.f, 0.f, 0.f}) - currentHeight, 0};
    glm::vec3 vec2{0, getHeightOnSmoothSurfaceWithGridPos(gridPosition + glm::vec3{0.f, 0.f, 1.f}) - currentHeight, squareSize};
    return glm::normalize(glm::cross(vec2, vec1));
  }

  glm::vec3 calculateNormalOnSmoothSurfaceWithWorldPos(glm::vec3 worldPosition)
  {
    // convert xzWorldPosition to grid coordinates
    worldPosition.x /= squareSize;
    worldPosition.z /= squareSize;
    return calculateNormalOnSmoothSurfaceWithGridPos(worldPosition);
  }

  glm::vec3 getNormalOnSmoothSurfaceWithGridPos(glm::vec3 gridPosition)
  {
    return vertices[(gridXSquareCount + 1) * floor(gridPosition[0]) + floor(gridPosition[2])].Normal;
  }

  glm::vec3 getNormalOnSmoothSurfaceWithWorldPos(glm::vec3 worldPosition)
  {
    worldPosition.x /= squareSize;
    worldPosition.z /= squareSize;
    return getNormalOnSmoothSurfaceWithGridPos(worldPosition);
  }

  float smoothMin(float a, float b, float k)
  {
    k = max(0.f, k);
    float h = max(0.f, min(1.f, (b - a + k) / (2.f * k)));
    return a * h + b * (1 - h) - k * h * (1 - h);
  }

  float smoothMax(float a, float b, float k)
  {
    k = min(0.f, -k);
    float h = max(0.f, min(1.f, (b - a + k) / (2 * k)));
    return a * h + b * (1 - h) - k * h * (1 - h);
  }

  float calculateCraterDepth(Crater craterInfo, glm::vec2 position)
  {
    float x = glm::length(position - craterInfo.center) / max(craterInfo.radius, 0.0001f);

    float cavity = x * x - 1;
    float rimX = min(x - 1.f - craterInfo.rimWidth, 0.f);
    float rim = craterInfo.rimSteepness * rimX * rimX;

    float craterShape = smoothMax(cavity, craterInfo.floor, craterInfo.smoothness);
    craterShape = smoothMin(craterShape, rim, craterInfo.smoothness);
    return craterShape * craterInfo.radius;
  }

  // Deforms the y-values of the vertices a given grid mesh with a variety of techniques (mainly perlin noise) and calculate new normals
  void generateLunarSurface(unsigned int xSquareCount, unsigned int zSquareCount, float squareLength, const char *texturePath, float textureScale, glm::vec2 translate)
  {
    generateGridPlane(xSquareCount, zSquareCount, squareLength, texturePath, textureScale, translate);

    // generate the biome features, for now over the entire 'infinite' plane, only 1 specific 'biome' is generated
    Perlin p;

    /* #region base surface */
    for (size_t i = 0; i < vertices.size(); i++)
    {
      // perlin maps
      vertices[i].Position.y += p.noise(
        vertices[i].Position.x,
        vertices[i].Position.z,
        0.738f,
        9.f,
        20.f,
        0);

      vertices[i].Position.y += p.noise(
        vertices[i].Position.x,
        vertices[i].Position.z,
        0.738f,
        5.f,
        10.f,
        1);

      vertices[i].Position.y += p.noise(
        vertices[i].Position.x,
        vertices[i].Position.z,
        0.738f,
        1.f,
        4.f,
        2);

      // activation function
      vertices[i].Position.y = 1.4f / (1 + exp(-2.5f * (vertices[i].Position.y - 7.5f)));

      vertices[i].Position.y += p.noise(
                                  vertices[i].Position.x,
                                  vertices[i].Position.z,
                                  0.738f,
                                  0.3f,
                                  1.f,
                                  3) *
                                p.noise(
                                  vertices[i].Position.x,
                                  vertices[i].Position.z,
                                  0.738f,
                                  0.5f,
                                  3.f,
                                  4);

      vertices[i].Position.y += p.noise(
        vertices[i].Position.x,
        vertices[i].Position.z,
        0.738f,
        0.015f,
        0.3f,
        5);
    }

    /* #endregion */

    /* #region  craters */
    // base crater properties
    Crater craterInfo{};
    craterInfo.center = glm::vec2{2.f, 2.f};
    craterInfo.floor = 0.003f;
    craterInfo.radius = 0.4f;
    craterInfo.rimSteepness = 0.2f;
    craterInfo.rimWidth = 1.8f;
    craterInfo.smoothness = 5.f;

    for (int c = 0; c < 22; c++)
    {
      // setting crater properties with even-distribution random function and constant term
      craterInfo.center = glm::vec2{30.f * rand() / RAND_MAX - 10.f, 30.f * rand() / RAND_MAX - 10.f};

      // non-linear crater size function
      float k = pow(1 - 0.75f, 3);
      float x = ((float)rand() / (float)RAND_MAX);
      craterInfo.radius = 0.11f + 1.85f * (x * k) / (x * k - x + 1.f);

      craterInfo.rimWidth = 0.4f + 10.f * rand() / RAND_MAX * (0.8f * craterInfo.radius + 0.7f);

      // non-linear crater rimwidth function
      k = pow(1 - 0.8f, 3);
      x = ((float)rand() / (float)RAND_MAX);
      craterInfo.rimSteepness = 0.01f + 0.2f * (x * k) / (x * k - x + 1.f);

      // spawn crater
      for (size_t i = 0; i < vertices.size(); i++)
      {
        vertices[i].Position.y += calculateCraterDepth(craterInfo, glm::vec2{vertices[i].Position.x, vertices[i].Position.z});
      }
    }
    /* #endregion */

    /* #region  update normals */
    for (int z = 0; z < zSquareCount; z++)
    {
      for (int x = 0; x < xSquareCount; x++)
      {
        vertices[(xSquareCount + 1) * x + z].Normal = calculateNormalOnSmoothSurfaceWithGridPos(glm::vec3{x, 0, z});
      }
    }

    // go over positive side edges
    for (int z = 0; z < zSquareCount + 1; z++)
    {
      vertices[(xSquareCount + 1) * xSquareCount + z].Normal = vertices[(xSquareCount + 1) * (xSquareCount - 1) + z].Normal;
    }
    for (int x = 0; x < xSquareCount + 1; x++)
    {
      vertices[(xSquareCount + 1) * x + zSquareCount].Normal = vertices[(xSquareCount + 1) * x + zSquareCount - 1].Normal;
    }
    /* #endregion */

    setupMesh();
  }

  ~BaseLunarSurface() {
    glBindVertexArray(VAO);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindVertexArray(0);
    
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteVertexArrays(1, &VAO);
  }
private:
};

class Rocks : public Mesh {
public:
  // default constructor
  Rocks() {};

  unsigned int addIcoSphereSubdivision(glm::vec3 pos1, glm::vec3 pos2) {
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::normalize(pos1 + pos2);
    return vertices.size() - 1;
  }

  void generateIcoSphere(unsigned int recursionDepth = 0) {
    unsigned int vertexCountBefore = vertices.size();
    // generate vertices
    float t = (1.f + sqrt(5.f)) / 2.f;

    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{-1.f,  t, 0};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 1.f,  t, 0};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{-1.f, -t, 0};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 1.f, -t, 0};

    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 0, -1.f,  t};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 0,  1.f,  t};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 0, -1.f, -t};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ 0,  1.f, -t};

    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ t, 0, -1.f};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{ t, 0,  1.f};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{-t, 0, -1.f};
    vertices.push_back(Vertex{});
    vertices[vertices.size() - 1].Position = glm::vec3{-t, 0, 1.f};

    // normalize to make it an aproximation from below of the unit circle
    for (size_t i = vertexCountBefore; i < vertices.size(); i++)
    {
      vertices[i].Position = glm::normalize(vertices[i].Position);
    }

    // generate indices
    std::vector<unsigned int> newIndices = vector<unsigned int>{
      0, 11, 5,
      0, 5, 1,
      0, 1, 7,
      0, 7, 10, 
      0, 10, 11,

      1, 5, 9,
      5, 11, 4, 
      11, 10, 2,
      10, 7, 6,
      7, 1, 8, 

      3, 9, 4,
      3, 4, 2, 
      3, 2, 6,
      3, 6, 8,
      3, 8, 9,

      4, 9, 5,
      2, 4, 11,
      6, 2, 10,
      8, 6, 7,
      9, 8, 1,
    };
    
    // account for the fact that there might already have been rocks added to the vertex list beforehand
    for (size_t i = 0; i < newIndices.size(); i++)
    {
      newIndices[i] += vertexCountBefore;
    }
  
    // calculate higher detail level version
    for (unsigned int i = 0; i < recursionDepth; i++)
    {
      vector<unsigned int> indicesPlaceHolder;

      for (unsigned int j = 0; j < newIndices.size(); j += 3)
      {
        // add the resulting points from subdividing into the vertex array
        unsigned int p1 = addIcoSphereSubdivision(vertices[newIndices[j]].Position, vertices[newIndices[j + 1]].Position);
        unsigned int p2 = addIcoSphereSubdivision(vertices[newIndices[j + 1]].Position, vertices[newIndices[j + 2]].Position);
        unsigned int p3 = addIcoSphereSubdivision(vertices[newIndices[j + 2]].Position, vertices[newIndices[j]].Position);

        // add the indices for the new faces
        indicesPlaceHolder.push_back(newIndices[j]);
        indicesPlaceHolder.push_back(p1);
        indicesPlaceHolder.push_back(p3);

        indicesPlaceHolder.push_back(newIndices[j + 1]);
        indicesPlaceHolder.push_back(p2);
        indicesPlaceHolder.push_back(p1);

        indicesPlaceHolder.push_back(newIndices[j + 2]);
        indicesPlaceHolder.push_back(p3);
        indicesPlaceHolder.push_back(p2);

        indicesPlaceHolder.push_back(p1);
        indicesPlaceHolder.push_back(p2);
        indicesPlaceHolder.push_back(p3);
      }

      newIndices = indicesPlaceHolder;
    }
  
    indices.insert(indices.end(), newIndices.begin(), newIndices.end());
  }
  
  // determines rock base diameter according to a given diameter distribution function. Based on analysis of the Chang'E-3 landing site
  // https://doi.org/10.1016/j.pss.2015.11.012
  // =============================================
  // std::vector<glm::vec3>& rockPositions : the set of 3d world positions of all rocks in the environment
  std::vector<float> generateRockSizes(std::vector<glm::vec3>& rockPositions) {
    std::vector<float> sizes;

    float ePow = std::exp(7.f/5.f);
    float a = 1.f/14.f;

    for (size_t i = 0; i < rockPositions.size(); i++)
    {
      float x = (float)rand() / (float)RAND_MAX + 0.0001f;

      sizes.push_back(a * std::log(ePow/(3*x)));
    }

    return sizes;
  }

  std::vector<float> generateRockSizes(int count) {
    std::vector<float> sizes;

    float ePow = std::exp(7.f/5.f);
    float a = 1.f/14.f;

    for (size_t i = 0; i < count; i++)
    {
      float x = (float)rand() / (float)RAND_MAX + 0.0001f;

      sizes.push_back(a * std::log(ePow/(3*x)));
    }

    return sizes;
  }
  

  void generateRock(glm::vec3 offset = glm::vec3{0.f}, glm::vec3 scale = glm::vec3{1.f}, unsigned int recursionDepth = 0) {
    srand(time(nullptr));

    unsigned int vertexCountBeforeRockGen = vertices.size();
    unsigned int indexCountBeforeRockGen = indices.size();

    generateIcoSphere(recursionDepth);
    
    unsigned int offsetIndex = p.randOffsetPairVec.size();

    // deform icosphere into something resembling a rock
    for (size_t i = vertexCountBeforeRockGen; i < vertices.size(); i++)
    {
      vertices[i].Position *= 0.5f + p.noise(vertices[i].Position.x, vertices[i].Position.y, vertices[i].Position.z, 1.f, 1.2f, offsetIndex);
      vertices[i].Position *= 0.9f + p.noise(vertices[i].Position.x, vertices[i].Position.y, vertices[i].Position.z, 0.2f, 0.3f, offsetIndex);
      vertices[i].Position *= scale;
      vertices[i].Position += offset;
    }

    // calculate normals
    for (size_t i = indexCountBeforeRockGen; i < indices.size(); i+=3)
    {
      glm::vec3 vec1 = vertices[indices[i + 1]].Position - vertices[indices[i]].Position;
      glm::vec3 vec2 = vertices[indices[i + 2]].Position - vertices[indices[i]].Position;
      glm::vec3 normal = glm::normalize(glm::cross(vec1, vec2));

      vertices[indices[i]].Normal += normal;
      vertices[indices[i + 1]].Normal += normal;
      vertices[indices[i + 2]].Normal += normal;
    }
    for (size_t i = vertexCountBeforeRockGen; i < vertices.size(); i++)
    {
      vertices[i].Normal = glm::normalize(vertices[i].Normal);
    }
    
    setupMesh();
  }

  void generateRocksOnSurface(BaseLunarSurface& surface) {
    // generate rock positions
    glm::vec3 smallerSurfaceCorner = glm::vec3{surface.offset.x, 0, surface.offset.y};
    glm::vec3 largerSurfaceCorner = glm::vec3{surface.squareSize * surface.gridXSquareCount, 0,  surface.squareSize * surface.gridZSquareCount} + smallerSurfaceCorner;

    // according the CHANG'E-3 landing site analysis, the rock density there is 0.447 rocks / m^2 with a diameter over 0.05 m 
    // https://doi.org/10.1016/j.pss.2015.11.012
    float surfaceArea = surface.squareSize * surface.gridXSquareCount * surface.squareSize * surface.gridZSquareCount;
    std::vector<glm::vec2> XZRockPositions = placePointsRandom((int)(surfaceArea * 0.447f), surface.squareSize * surface.gridXSquareCount, surface.squareSize * surface.gridZSquareCount, smallerSurfaceCorner);
    std::vector<glm::vec3> rockPositions;

    // retrieve the height of the lunar surface at the rock world position
    for (size_t i = 0; i < XZRockPositions.size(); i++)
    {
      rockPositions.push_back(glm::vec3{XZRockPositions[i][0], surface.getHeightOnSmoothSurfaceWithWorldPos(glm::vec3{XZRockPositions[i][0], 0, XZRockPositions[i][1]}), XZRockPositions[i][1]});
    }

    // generate base rock sizes
    std::vector<float> sizes = generateRockSizes(rockPositions);

    // generate rocks
    for (size_t i = 0; i < rockPositions.size(); i++)
    {
      generateRock(rockPositions[i], glm::vec3{sizes[i]/2.f}, 4);
    }
  }

private:
  Perlin p;

  std::vector<glm::vec2> placePointsRandom(int count, float w, float l, glm::vec3 offset = glm::vec3{0.f}) {
    std::vector<glm::vec2> points;

    for (int i = 0; i < count; i++)
    {
      float x = rand() / (float)RAND_MAX * w;
      float z = rand() / (float)RAND_MAX * l;
      points.push_back(glm::vec2{x, z} + glm::vec2{offset.x, offset.z});
    }
    return points;
  }

  std::vector<glm::vec2> relaxPoints(std::vector<glm::vec2> points, int steps, float min_distance, float strength) {
    for (int v = 0; v < steps; v++)
    {
      for (size_t i = 0; i < points.size(); i++)
      {
        for (size_t j = 0; j < points.size(); j++) {
          if( i ==j) { continue; }

          glm::vec2 point1 = points[i];
          glm::vec2 point2 = points[j];

          // cheap check to reserve relatively expensive euclidian distance calculation for those pairs that need it
          bool quickTestResult = abs(point1.x - point2.x) < min_distance && abs(point1.y - point2.y) < min_distance;

          if(quickTestResult && glm::distance(point1, point2) < min_distance) {
            glm::vec2 differenceNormal = glm::normalize(point2 - point1);

            point1.x += differenceNormal.x * strength;
            point1.y += differenceNormal.y * strength;
            point2.x += differenceNormal.x * strength;
            point2.y += differenceNormal.y * strength;
          }

          points[i] = point1;
          points[j] = point2;
        }
      }
    }
    return points;
  }

  float getScaleForPoint(glm::vec2 point, std::vector<glm::vec2> points, float boundaryDistance) {
    float scale = 1.f;

    for (size_t i = 0; i < points.size(); i++)
    {
      bool quickTestResult = abs(point.x - points[i].x) < boundaryDistance && abs(point.y - points[i].y) < boundaryDistance;

      if(quickTestResult && glm::distance(point, points[i]) < boundaryDistance) {
        scale += 1.f / (glm::distance(point, points[i]) + 1.f);
      }
    }
    scale *= ((float)rand() / (float)RAND_MAX + 0.2f);

    return scale;
  }

};

class LunarSurface {
public:
  BaseLunarSurface baseSurfaceHighDetail;
  BaseLunarSurface baseSurfaceLowDetail[8];
  Rocks rocks;

  // default constructor
  LunarSurface() {}

  void generateSmoothSurface() {
    baseSurfaceHighDetail.generateLunarSurface(200, 200, 0.05f, "../resources/textures/sandDiffuseMap.png", 0.004f, glm::vec2{0.f, 0.f});

    glm::vec2 surfacePositions[8] = {
      glm::vec2{10.f, 0.f},
      glm::vec2{-10.f, 0.f},
      glm::vec2{10.f, 10.f},
      glm::vec2{10.f, -10.f},
      glm::vec2{-10.f, 10.f},
      glm::vec2{-10.f, -10.f},
      glm::vec2{0.f, 10.f},
      glm::vec2{0.f, -10.f}
    };

    for (unsigned int i = 0; i < 8; i++)
    {
      baseSurfaceLowDetail[i].generateLunarSurface(100, 100, 0.1f, "../resources/textures/sandDiffuseMap.png", 0.004f, surfacePositions[i]);
    }

    rocks.generateRocksOnSurface(baseSurfaceHighDetail);
  }

  void draw(Shader& shader, int FBO = -1) {
    glm::mat4 modelMatrix;

    modelMatrix = baseSurfaceHighDetail.getModelMatrix();
    shader.setMat4("model", modelMatrix);
    baseSurfaceHighDetail.draw(shader, FBO);

    if(FBO != -1) { return; }
    for (unsigned int i = 0; i < 8; i++)
    {
      modelMatrix = baseSurfaceLowDetail[i].getModelMatrix();
      shader.setMat4("model", modelMatrix);
      baseSurfaceLowDetail[i].draw(shader, FBO);
    }
    modelMatrix = rocks.getModelMatrix();
    shader.setMat4("model", modelMatrix);
    rocks.draw(shader, FBO);
  }

private:
};

#endif