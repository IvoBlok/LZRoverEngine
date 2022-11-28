#ifndef MESH_H
#define MESH_H

#include <glad/glad.h> // holds all OpenGL type declarations

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "shaders/shader.h"
#include "../external/Worley.h"

#include <string>
#include <vector>
using namespace std;

#define MAX_BONE_INFLUENCE 4

glm::mat4 getModelMatrixFromAngle(glm::vec3 position, glm::vec3 up, float angle)
{
    glm::vec3 front = glm::normalize(glm::cross(up, glm::vec3{0.f, 0.f, -1.f}));
    front = glm::rotate(front, glm::radians(angle), up);
    return glm::lookAt(position, position + front, up);
}

unsigned int TextureFromFileAbsPath(const char *path, bool gamma)
{
    string filename = string(path);

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

struct Crater
{
    glm::vec2 center;
    float radius;
    float floor;
    float smoothness;
    float rimSteepness;
    float rimWidth;
};

struct Vertex
{
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal{0.f, 1.f, 0.f};
    // texCoords
    glm::vec2 TexCoords{0.f, 0.f};
    // tangent
    glm::vec3 Tangent{0.f, 0.f, 0.f};
    // bitangent
    glm::vec3 Bitangent{0.f, 0.f, 0.f};
    // bone indexes which will influence this vertex
    int m_BoneIDs[MAX_BONE_INFLUENCE];
    // weights from each bone
    float m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture
{
    unsigned int id;
    string type;
    string path;
};

class Mesh
{
protected:
  char path[256];

public:
  // mesh Data
  glm::vec3 position{0.f, 0.f, 0.f};
  glm::vec3 up{0.f, 1.f, 0.f};
  glm::vec3 front{1.f, 0.f, 0.f};
  glm::vec3 scale{1.f};

  vector<Vertex> vertices;
  vector<unsigned int> indices;
  vector<Texture> textures;

  // render buffers
  unsigned int VBO, EBO, VAO;

  // default constructor
  Mesh() {}

  Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures)
  {
    this->vertices = vertices;
    this->indices = indices;
    this->textures = textures;

    // now that we have all the required data, set the vertex buffers and its attribute pointers.
    setupMesh();
  }

  virtual void loadTexture(const char *texturePath, std::string texture_type, int textureVectorIndex = -1)
  {
    const char *absPath = realpath(texturePath, path);
    Texture texture;
    texture.id = TextureFromFileAbsPath(absPath, false);
    texture.type = texture_type;

    if (textureVectorIndex < 0)
    {
      textures.push_back(texture);
    }
    else
    {
      textures[textureVectorIndex] = texture;
    }
  }

  virtual glm::mat4 getModelMatrix()
  {
    glm::vec3 right = glm::cross(front, up);
    glm::mat4 model = glm::mat4{1.f};

    model[0] = glm::vec4(front, 0.f);
    model[1] = glm::vec4(up, 0.f);
    model[2] = glm::vec4(right, 0.f);
    model[3] = glm::vec4(position, 1.f);

    model = glm::scale(model, scale);
    return model;
  }

  // render the mesh
  virtual void draw(Shader &shader, int FBO = -1)
  {
    shader.use();
    // bind appropriate textures
    unsigned int diffuseNr = 1;
    unsigned int specularNr = 1;
    unsigned int normalNr = 1;
    unsigned int heightNr = 1;
    for (unsigned int i = 0; i < textures.size(); i++)
    {
      glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
      // retrieve texture number (the N in diffuse_textureN)
      string number;
      string name = textures[i].type;
      if (name == "texture_diffuse")
        number = std::to_string(diffuseNr++);
      else if (name == "texture_specular")
        number = std::to_string(specularNr++); // transfer unsigned int to string
      else if (name == "texture_normal")
        number = std::to_string(normalNr++); // transfer unsigned int to string
      else if (name == "texture_height")
        number = std::to_string(heightNr++); // transfer unsigned int to string

      // now set the sampler to the correct texture unit
      glUniform1i(glGetUniformLocation(shader.ID, (name + number).c_str()), i);
      // and finally bind the texture
      glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }

    // draw mesh
    glBindVertexArray(VAO);
    if(FBO != -1) {
      glBindFramebuffer(GL_FRAMEBUFFER, FBO);
    }
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
    if(FBO != -1) {
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    glBindVertexArray(0);

    // always good practice to set everything back to defaults once configured.
    glActiveTexture(GL_TEXTURE0);
  }

  // initializes all the buffer objects/arrays
  virtual void setupMesh()
  {
    // create buffers/arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // A great thing about structs is that their memory layout is sequential for all its items.
    // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
    // again translates to 3/2 floats which translates to a byte array.
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, Normal));
    // vertex texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, TexCoords));
    // vertex tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, Tangent));
    // vertex bitangent
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, Bitangent));
    // ids
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex), (void *)offsetof(Vertex, m_BoneIDs));

    // weights
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, m_Weights));
    glBindVertexArray(0);
  }

};

#endif
