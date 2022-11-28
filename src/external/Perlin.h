/**
 * Perlin.h
 * v. 1.0.0
 * 
 * Definition for Perlin class. An instantiated Perlin object can generate smoothed Perlin noise by calling the noise() function.
 *
 * Copyright Chris Little 2012
 * Author: Chris Little
 */

#ifndef _PERLIN_H_
#define _PERLIN_H_

#include <utility>
#include <vector>
#include <iostream>
#include <glm/glm.hpp>

class Perlin {
public:
	// random offset list
	std::vector<glm::vec3> randOffsetPairVec;

	Perlin(unsigned int seed = 0);
	~Perlin();

	// Generates a Perlin (smoothed) noise value between 0 and height, at the given 3D position.
	float noise(float sample_x, float sample_y, float sample_z, float height = 1.f, float scale = 1.f, unsigned int offsetIndex = 0);

private:
	int *p; // Permutation table
	// Gradient vectors
	float *Gx;
	float *Gy;
	float *Gz;
};

#endif