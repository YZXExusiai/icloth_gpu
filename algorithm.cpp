#include"algorithm.h"
void Nature_model(double step, Mesh* mesh, glm::vec3 g,double damping) {
	for (int i = 0; i < mesh->verts.size();i++) {
		mesh->verts[i]->v *= damping;
		mesh->verts[i]->v += g * (float)mesh->verts[i]->mass_inv;
		mesh->verts[i]->x0 = mesh->verts[i]->x;
		mesh->verts[i]->x += (float)step * mesh->verts[i]->v;
	}
}
void PBD_model(double step,Mesh *mesh) {
	std::vector<glm::vec3> Sum_V(mesh->verts.size());
	std::vector<int> Sum_N(mesh->verts.size());
	for (int i = 0; i < mesh->edges.size(); i++) {
		glm::vec3 a = mesh->edges[i]->ver[0]->x0;
		glm::vec3 b = mesh->edges[i]->ver[1]->x0;
		Sum_V[mesh->edges[i]->ver[0]->index] += 0.5f * (a + b + (float)mesh->edges[i]->len * glm::normalize(a - b));
		Sum_N[mesh->edges[i]->ver[0]->index] ++;
		Sum_V[mesh->edges[i]->ver[1]->index] += 0.5f * (a + b - (float)mesh->edges[i]->len * glm::normalize(a - b));
		Sum_N[mesh->edges[i]->ver[1]->index] ++;
	}
	//通过移动的位置反推速度以及使用Jacobi法更新
	for (int i = 0; i < Sum_V.size(); i++) {
		if (i == 0 || i == 10 || i == 110 || i == 120)continue;
		float t_inv = 1 / step;
		glm::vec3 temp = (0.2f * mesh->verts[i]->x0 + Sum_V[i]) / (0.2f + Sum_N[i]);
		mesh->verts[i]->v += t_inv * (temp - mesh->verts[i]->x0);
		mesh->verts[i]->x0 = mesh->verts[i]->x;
		mesh->verts[i]->x = temp;
	}
}