#include "spatialHash.h"

aabb Getbox(Face* f) {
	glm::vec3 a = f->ver[0]->x;
	glm::vec3 b = f->ver[1]->x;
	glm::vec3 c = f->ver[2]->x;
	float Xmax = std::max(a.x, std::max(b.x, c.x));
	float Xmin = std::min(a.x, std::min(b.x, c.x));
	float Ymax = std::max(a.y, std::max(b.y, c.y));
	float Ymin = std::min(a.y, std::min(b.y, c.y));
	float Zmax = std::max(a.z, std::max(b.z, c.z));
	float Zmin = std::min(a.z, std::min(b.z, c.z));
	aabb temp;
	temp.Xmax = Xmax;
	temp.Xmin = Xmin;
	temp.Ymax = Ymax;
	temp.Ymin = Ymin;
	temp.Zmax = Zmax;
	temp.Zmin = Zmin;
	return temp;
}
spatialHash CreateHash(int num) {
	spatialHash temp;
	temp.cellnum = num;
	std::vector<HashCell> tp(num * num * num);
	temp.Hash = tp;
	return temp;
}
void SetHashCell(spatialHash& hash, Tri* t, float length) {
	float len = (length + 0.01f) / hash.cellnum;
	float Xmax = t->box.Xmax;
	float Ymax = t->box.Ymax;
	float Zmax = t->box.Zmax;
	float Xmin = t->box.Xmin;
	float Ymin = t->box.Ymin;
	float Zmin = t->box.Zmin;
	int Xleft = int((Xmin + length / 2.0) / len);
	int Xright = int((Xmax + length / 2.0) / len);
	int Yleft = int((Ymin + length / 2.0) / len);
	int Yright = int((Ymax + length / 2.0) / len);
	int Zleft = int((Zmin + length / 2.0) / len);
	int Zright = int((Zmax + length / 2.0) / len);
	t->f->setNormal();//刷新法向锥
	for (int i = Xleft; i <= Xright; i++) {
		for (int j = Yleft; j <= Yright; j++) {
			for (int k = Zleft; k <= Zright; k++) {
				//std::cout << Xmax <<" "<< Xmin << " "<<len<<std::endl;
				//std::cout << t.adj.size() << std::endl;
				int index = k * hash.cellnum * hash.cellnum + j * hash.cellnum + i;
				//std::cout << index << std::endl;
				bool isset = true;
				if (hash.Hash[index].cell.size() == 0) {
					hash.Hash[index].cell.push_back(t);
				}
				else {
					for (int p = 0; p < hash.Hash[index].cell.size(); p++) {
						if (!isset)break;
						//获取不同物体的标识
						int temp = hash.Hash[index].cell[p]->obj;
						for (int tp = 0; tp < t->adj.size(); tp++) {
							//如果是同一OBJ的三角面片就剔除
							if (temp == t->obj) {
								isset = false;
								break;
							}
						}
						/*
						//获取哈希格中已经有的三角面片
						int temp = hash.Hash[index].cell[p]->id;
						glm::vec3 temp_n = hash.Hash[index].cell[p]->f->normal;
						for (int tp = 0; tp < t->adj.size(); tp++) {
							//如果是邻接的三角面片就剔除
							if (temp == t->adj[tp]) {
								isset = false;
								break;
							}
							//如果是近似法向锥的三角面片就剔除
							else if (glm::dot(temp_n, t->f->normal) > 0.8) {
								isset = false;
								break;
							}
						}
						*/
					}
					if (isset) {
						hash.Hash[index].cell.push_back(t);
					}
				}
			}
		}
	}
}
bool IsAdjacency(const Tri* a, const Tri* b) {
	if (a->f->ver[0] == b->f->ver[0] || a->f->ver[1] == b->f->ver[0] || a->f->ver[2] == b->f->ver[0] || 
		a->f->ver[0] == b->f->ver[1] || a->f->ver[1] == b->f->ver[1] || a->f->ver[2] == b->f->ver[1] ||
		a->f->ver[0] == b->f->ver[2] || a->f->ver[1] == b->f->ver[2] || a->f->ver[2] == b->f->ver[2]) {
		return true;
	}
	return false;
}
void SetAdjacency(std::vector<Tri*>& T) {
	//std::cout << T.size() << std::endl;
	for (int i = 0; i < T.size() - 1; i++) {
		for (int j = i + 1; j < T.size(); j++) {
			if (IsAdjacency(T[i], T[j])) {
				T[i]->adj.push_back(T[j]->id);
				T[j]->adj.push_back(T[i]->id);
			}
		}
	}
}
std::vector<HashCell> SperateTri(int num, std::vector<Tri*> T, float length) {
	spatialHash H = CreateHash(num);
	for (int i = 0; i < T.size(); i++) {
		SetHashCell(H, T[i],length);
	}
	for (int i = 0; i < H.Hash.size(); i++) {
		//如果该哈希格中三角面片数量不超过1，就跳过
		if (H.Hash[i].cell.size() <= 1) {
			continue;
		}
		//把哈希表中存在多个非邻接或近似法向锥的三角面片的格子选出
		else {
			H.CollisionArea.push_back(H.Hash[i]);
		}
	}
	std::cout<<"Hash Num:" << H.CollisionArea.size() << std::endl;
	return H.CollisionArea;
}
std::vector<HashCell> UpdateTriangle(int num, std::vector<Tri*> T, float length) {
	for (int i = 0; i < T.size();i++) {
		T[i]->box = Getbox(T[i]->f);
	}
	return SperateTri(num, T, length);//CollisionArea每次重新设置变量
}