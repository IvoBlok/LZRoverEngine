#ifndef _WORLEY_H_
#define _WORLEY_H_

#include <glm/glm.hpp>
#include <vector>

class Worley {
public:
    Worley();
	
    // random offset list
	std::vector<std::pair<float, float>> randOffsetPairVec;

    float noise(glm::vec3 pos, unsigned int offsetIndex = 0);
private:
    // for calculation sake this is a float datatype, but only assign it integer values
    float cellCount = 20.f;

    glm::vec2 getCellPoint(glm::vec2 cell);
};

#endif