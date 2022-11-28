#version 330
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoords;

out vec2 texCoords;
out vec3 normals;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    normals = mat3(transpose(inverse(model))) * aNormal;

    gl_Position = projection * view * model * vec4(aPos, 1.0);
    texCoords = aTexCoords;
}
