#version 330 core
out vec4 FragColor;

in vec2 texCoords;
in vec3 normals;

uniform sampler2D texture_diffuse;
uniform vec3 lightColor;
uniform vec3 lightDir;

void main()
{
    float ambient = 0.1;

    float diffuse = max(dot(normals, -lightDir), 0.0);

    FragColor = vec4((ambient + diffuse) * lightColor * texture2D(texture_diffuse, texCoords).xyz, 1.0);
}
