#include "simulation.h"
using namespace std;
//简易的模拟器定义，参数自己设置
Simulation initial_sim() {
	glm::vec3 g(0, -0.98, 0);
	Simulation s;
	s.damping = 0.98;
	s.gravity = g;
	s.step_time = 0.002;
	Collision c;
	s.collision = c;
	return s;
}
bool push_sim(Simulation& sim, Mesh* cloth) {
	if (cloth != nullptr) {
		SetAdjacency(cloth->tri);
		sim.procedual_mesh.push_back(cloth);
		//sim.collision.Tris.clear();
		for (auto tri : cloth->tri) {
			sim.collision.Tris.push_back(tri);
		}
		std::cout << sim.collision.Tris.size() << endl;
		return true;
	}
	return false;
}
bool push_sim_static(Simulation& sim, Mesh* block) {
	if (block != nullptr) {
		SetAdjacency(block->tri);
		sim.static_mesh.push_back(block);
		//sim.collision.Tris.clear();
		for (auto tri : block->tri) {
			sim.collision.Tris.push_back(tri);
		}
		std::cout << sim.collision.Tris.size() << endl;
		return true;
	}
	return false;
}
Mesh* initial_cloth(ImportedModel myModel)
{
	Mesh* cloth = new Mesh;
	vector<glm::vec3> node = myModel.getNode();
	vector<int> index = myModel.getFaceId();
	int num_v = myModel.getNodeNum();
	vector<glm::vec2> text = myModel.getTextureCoords();
	vector<glm::vec3> DrawVertex = myModel.getVertices();
	int faceNum = myModel.getNumVertices() / 3;
	for (int i = 0; i < num_v;i++) {
		Vertice *v = new Vertice(node[i],i,0);
		if(i == 0||i==10 || i == 110 || i == 120)v->mass_inv = 0;
		else v->mass_inv = 1;
		cloth->verts.push_back(v);
	}
	cloth->index = index;
	cloth->texture = text;
	for (int i = 0; i < faceNum;i++) {
		Face *f = new Face(cloth->verts[index[3 * i]], cloth->verts[index[3 * i + 1]], cloth->verts[index[3 * i + 2]]);
		Tri* t = new Tri(f, i, 0);
		Edge *e1 = new Edge(cloth->verts[index[3 * i]], cloth->verts[index[3 * i + 1]]);
		Edge *e2 = new Edge(cloth->verts[index[3 * i + 1]], cloth->verts[index[3 * i + 2]]);
		Edge *e3 = new Edge(cloth->verts[index[3 * i]], cloth->verts[index[3 * i + 2]]);
		cloth->faces.push_back(f);
		cloth->tri.push_back(t);
		cloth->edges.push_back(e1);
		cloth->edges.push_back(e2);
		cloth->edges.push_back(e3);
	}
	//std::cout << cloth->verts.size() << endl;
	//std::cout << cloth->index.size() << endl;
	//std::cout << cloth->faces.size() << endl;
	//std::cout << cloth->texture.size() << endl;
	return cloth;
}
Mesh* initial_block(ImportedModel myModel) {
	Mesh* block = new Mesh;
	vector<glm::vec3> node = myModel.getNode();
	vector<int> index = myModel.getFaceId();
	int num_v = myModel.getNodeNum();
	vector<glm::vec2> text = myModel.getTextureCoords();
	vector<glm::vec3> DrawVertex = myModel.getVertices();
	int faceNum = myModel.getNumVertices() / 3;
	for (int i = 0; i < num_v; i++) {
		glm::vec3 downs(0,-0.7f,0);
		Vertice* v = new Vertice(node[i]+downs, i, 0);
		v->mass_inv = 1;
		block->verts.push_back(v);
	}
	block->index = index;
	block->texture = text;
	for (int i = 0; i < faceNum; i++) {
		Face* f = new Face( block->verts[index[3 * i]],  block->verts[index[3 * i + 1]],  block->verts[index[3 * i + 2]]);
		Tri* t = new Tri(f, i, 1);
		Edge* e1 = new Edge( block->verts[index[3 * i]],  block->verts[index[3 * i + 1]]);
		Edge* e2 = new Edge( block->verts[index[3 * i + 1]],  block->verts[index[3 * i + 2]]);
		Edge* e3 = new Edge( block->verts[index[3 * i]],  block->verts[index[3 * i + 2]]);
		block->faces.push_back(f);
		block->tri.push_back(t);
		block->edges.push_back(e1);
		block->edges.push_back(e2);
		block->edges.push_back(e3);
	}
	//std::cout <<  block->verts.size() << endl;
	//std::cout <<  block->index.size() << endl;
	//std::cout <<  block->faces.size() << endl;
	//std::cout <<  block->texture.size() << endl;
	return  block;
}
void test_demo(const Simulation& sim) {
	for (int i = 0; i < sim.procedual_mesh.size(); i++) {
		Nature_model(sim.step_time, sim.procedual_mesh[i], sim.gravity, sim.damping);
		for (int j = 0; j < 30; j++) {
			PBD_model(sim.step_time, sim.procedual_mesh[i]);
		}
	}
}
void drawModel(const Mesh* cloth)
{
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < cloth->faces.size(); i++)//循环遍历face信息
	{
		glTexCoord2f(cloth->texture[3 * i].s, cloth->texture[3 * i].t);
		glVertex3f(cloth->faces[i]->ver[0]->x.x, cloth->faces[i]->ver[0]->x.y, cloth->faces[i]->ver[0]->x.z);
		glTexCoord2f(cloth->texture[3 * i + 1].s, cloth->texture[3 * i + 1].t);
		glVertex3f(cloth->faces[i]->ver[1]->x.x, cloth->faces[i]->ver[1]->x.y, cloth->faces[i]->ver[1]->x.z);
		glTexCoord2f(cloth->texture[3 * i + 2].s, cloth->texture[3 * i + 2].t);
		glVertex3f(cloth->faces[i]->ver[2]->x.x, cloth->faces[i]->ver[2]->x.y, cloth->faces[i]->ver[2]->x.z);
	}
	glEnd();
}
void DynamicModel(Simulation& sim)
{
	test_demo(sim);
	sim.collision.ip.clear();//每次模拟清空碰撞域
	vector<Mesh*> Detect_mesh;
	for (int i = 0; i < sim.static_mesh.size(); i++) {
		Detect_mesh.push_back(sim.static_mesh[i]);
	}
	for (int i = 0; i < sim.procedual_mesh.size();i++) {
		Detect_mesh.push_back(sim.procedual_mesh[i]);
	}
	board_phase(sim.collision, Detect_mesh, 20, 2.01f);
	sim.collision.ip.push_back(Collision_response());
	for (int j = 0; j < sim.collision.ip.size(); j++) {
		narrow_phase(sim.collision, sim.collision.ip[j]);
		//cout << "CollisionNodeNum:" << sim.collision.ip[j]->nodes.size() << endl;
	}
	Collision_Resolve(sim.collision);
}