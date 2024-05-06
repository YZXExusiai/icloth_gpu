#pragma once
#include <stdarg.h>
#include <iostream>
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <string>
#include <vector>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include"ImportedModel.h"
#include "algorithm.h"
#include "geometry.h"
#include "collision.h"

struct Wind {
    double density;
    glm::vec3 velocity;
    double drag;
};
struct Simulation {
    double time;
    std::vector<Mesh*> procedual_mesh;
    std::vector<Mesh*> static_mesh;
    int step;
    double step_time;
    double damping;
    Collision collision;
    glm::vec3 gravity;
    Wind wind;
    double friction, obs_friction;
};
void prepare(Simulation& sim);
void advance_step(Simulation& sim);
Simulation initial_sim();
bool push_sim(Simulation &sim, Mesh* cloth);
bool push_sim_static(Simulation& sim, Mesh* block);
void test_demo(const Simulation& sim);
Mesh* initial_cloth(ImportedModel myModel);
Mesh* initial_block(ImportedModel myModel);
std::vector<glm::vec3> node_positions(const std::vector<Mesh*>& meshes);
//extern function
void drawModel(const Mesh* cloth);
void DynamicModel(Simulation& sim);
