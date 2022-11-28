#version 330 core
layout(location = 0) out vec3 colorOut;

out vec4 fragColor;
in vec3 worldPos;

uniform float maxX;
uniform float maxY;
uniform float maxZ;

void main()
{
    vec3 worldPosNormalized = 0.5 * vec3(worldPos.x / maxX, worldPos.y / maxY, worldPos.z / maxZ) + 0.5;

    fragColor = vec4(vec3(gl_FragCoord.z), 1.f);
    colorOut = worldPosNormalized;
}
