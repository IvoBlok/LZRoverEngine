#include "Worley.h"

#include <cstdlib>
#include <algorithm>
#include <math.h>

Worley::Worley() {
}

float randFromVec2(glm::vec2 value) {
    double a = sin(glm::dot(glm::vec2{value.y, value.x}, glm::vec2{12.9898, 78.233})) * 43758.5453;
    return a - floor(a);
}

glm::vec2 Worley::getCellPoint(glm::vec2 cell) {
    glm::vec2 cellBase = glm::vec2{cell.x / cellCount, cell.y / cellCount};
    float noise_x = randFromVec2(cell);
    float noise_y = randFromVec2(glm::vec2{cell.y, cell.x});
    glm::vec2 temp = cellBase + (glm::vec2{0.5f} + glm::vec2{1.5f * noise_x, 1.5f * noise_y});
    return glm::vec2{temp.x/(float)cellCount, temp.y/(float)cellCount};
}   

float Worley::noise(glm::vec3 pos, unsigned int offsetIndex) {
    if(offsetIndex >= randOffsetPairVec.size()){
        randOffsetPairVec.push_back(std::pair<float, float>{float(rand() * 50 / (RAND_MAX/2)), float(rand() * 50 / (RAND_MAX/2))});
		offsetIndex = randOffsetPairVec.size() - 1;
    }
    pos.x += randOffsetPairVec[offsetIndex].first;
    pos.z += randOffsetPairVec[offsetIndex].second;

    pos.x += 110 * pos.y;

    glm::vec2 cellCoord{pos.x * cellCount, pos.z * cellCount};
    float dist = 1.f;

    // search the close cells
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            glm::vec2 cellPoint = getCellPoint(cellCoord + glm::vec2{i - 2, j - 2});
            dist = std::min(dist, glm::length(cellPoint - glm::vec2{pos.x, pos.z}));
        }
    }
    
    dist /= glm::length(glm::vec2{1.f/cellCount});
    dist = 1.f - dist;
    return dist;
}