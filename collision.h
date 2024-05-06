#pragma once
#include <string>
#include <vector>
#include <set>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include "spatialHash.h"
#include "optimization.h"

struct impact {
	int type;//type=0为VF碰撞，type=1为EE碰撞
	double t;
	glm::vec3 n;
	Vertice* node[4];
	double w[4];
	impact() {}
	impact(int type, const Vertice* n0, const Vertice* n1, const Vertice* n2, const Vertice* n3) {
		node[0] = (Vertice*)n0;
		node[1] = (Vertice*)n1;
		node[2] = (Vertice*)n2;
		node[3] = (Vertice*)n3;
	}
};
struct impactZone {
	std::vector<Vertice*> nodes;//碰撞域所有节点
	std::vector<impact> impacts;
	bool active = true;
};
struct Collision {
	std::vector<HashCell> CollisionArea;//boardphase下产生的hash碰撞格
	std::vector<Tri*> Tris;//boardphase下产生的Tri组
	std::vector<Tri*> CollisionTri;//narrowphase下产生的Tri组
	std::vector<impactZone*> ip;//impact zone
	std::vector<Constraint*> Cx;//约束组
	Collision() {}
};
glm::vec3 position(const Vertice* v, double t);
void board_phase(Collision &c,const std::vector<Mesh*> m,int num, float length);
void narrow_phase(Collision& c, impactZone* ip);
void Collision_Solve(Collision& c);
void Collision_Resolve(Collision& c);
impactZone* Collision_response();
double minGx(const std::vector<glm::vec3>& Vnext);
double min_target(Collision& c,const std::vector<glm::vec3>& Vnext);
std::vector<glm::vec3> node_positions(std::vector<Vertice*> nodes);
std::vector<glm::vec3> node_grad(std::vector<Vertice*> nodes);
void CreateNode(Tri *t1, Tri *t2, impact& i, impactZone* ip,Collision &c);
void CreateImpact(Collision &c, impactZone* ip);
void CreateNodeSeq(Collision& c, impactZone* ip);
void SetConstraint(Collision& c);
void SetTriConstraint(Collision& c, const Tri *x, const Tri *y);
void SetCvf(Collision& c,Vertice* x, Vertice* y, Vertice*z, Vertice*p);
void SetCee(Collision& c,Vertice* x1, Vertice* x2, Vertice* y1, Vertice* y2);
double Gradient_decent(Collision &c,int index);
void Next_step(std::vector<glm::vec3>& n, const std::vector<glm::vec3>& grad, double alpha);
bool collision_test(int type, const Vertice* n0, const Vertice* n1, const Vertice* n2, const Vertice* n3, impact& ip);//三角面片ccd碰撞检测
