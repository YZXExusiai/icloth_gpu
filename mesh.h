#pragma once
#include <glm\glm.hpp>
#include <vector>

struct Vertice;
struct Face;
struct Edge;
struct Node;

struct Vertice {
	int label;
	glm::vec3 y; // plastic embedding
	glm::vec3 x;//current postion
	glm::vec3 x0;//pre postion
	glm::vec3 v;//velocity
	int index;
	glm::vec3 normal;
	double area;
	double mass_inv;
	glm::vec3 acceleration;
	std::vector<Edge*> adj_edge;
	Vertice(){}
	explicit Vertice(const glm::vec3 x,int label = 0) {
		this->label = label;
		this->y = x;
		this->x = x;
		this->x0 = x;
		this->v = glm::vec3(0, 0, 0);
	}
	explicit Vertice(const glm::vec3 x, int id,int label) {
		this->label = label;
		this->y = x;
		this->x = x;
		this->x0 = x;
		this->v = glm::vec3(0, 0, 0);
		this->index = id;
	}
};

struct Edge {
	Vertice* ver[2];
	int label;
	Face* adj_face[2];
	int index;
	double theta;// actual dihedral angle
	double theta_ideal, damage; // rest dihedral angle, damage parameter
	double reference_angle; // just to get sign of dihedral_angle() right
	double len;
	Edge(){}
	explicit Edge(Vertice* v0, Vertice* v1, int label = 0) {
		this->label = label;
		ver[0] = v0;
		ver[1] = v1;
		len = (double)glm::distance(v0->x,v1->x);
	}
};

struct Face {
	Vertice* ver[3];
	int label;
	Edge* adj_edge[3];
	int index;
	glm::vec3 normal;
	double area;
	double mass_inv;
	glm::mat2x2 Dm;// deform matrix(for FEM)
	glm::mat2x2 Dm_inv;
	glm::mat2x2 S_plastic;// plastic strain
	Face() {}
	explicit Face(Vertice* v0, Vertice* v1, Vertice* v2, int label = 0) {
		this->label = label;
		ver[0] = v0;
		ver[1] = v1;
		ver[2] = v2;
		//求法向量
		this->normal = glm::normalize(glm::cross(v1->x - v0->x, v2->x - v0->x));
	}
	void setNormal() {
		this->normal = glm::normalize(glm::cross(this->ver[1]->x - this->ver[0]->x, this->ver[2]->x - this->ver[0]->x));
	}
};
struct aabb {
	float Xmax;
	float Xmin;
	float Ymax;
	float Ymin;
	float Zmax;
	float Zmin;
};

struct Tri {
	Face* f;
	bool mark = false;
	aabb box;
	int obj;
	int id;
	std::vector<int> adj;//邻接的三角面片的id集合
	Tri(Face *temp_f,int i,int obj) {
		this->f = temp_f;
		this->id = i;
		this->obj = obj;
	}
};

struct Mesh {
	std::vector<Vertice*> verts;
	std::vector<Edge*> edges;
	std::vector<Face*> faces;
	std::vector<Tri*> tri;
	std::vector<glm::vec2> texture;
	std::vector<int> index;
};
