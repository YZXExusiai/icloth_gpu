#pragma once
#include <vector>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include"mesh.h"
void Nature_model(double step, Mesh* mesh, glm::vec3 g, double damping);
void PBD_model(double step, Mesh* mesh);