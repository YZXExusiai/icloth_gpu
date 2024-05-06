#pragma once
#include <iostream>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include "mesh.h"
#include "constraint.h"
struct HashCell {
	std::vector<Tri*> cell;
};
struct spatialHash {
	std::vector<HashCell> Hash;
	std::vector<HashCell> CollisionArea;
	int cellnum = 20;//空间哈希表中每个方向被分成多少块,取决于三角面片大小
};
aabb Getbox(Face* f);
spatialHash CreateHash(int num);
void SetHashCell(spatialHash& hash, Tri* t,float length);
bool IsAdjacency(const Tri *a, const Tri *b);
void SetAdjacency(std::vector<Tri*> &T);
std::vector<HashCell> SperateTri(int num, std::vector<Tri*> T, float length);
std::vector<HashCell> UpdateTriangle(int num, std::vector<Tri*> T, float length);