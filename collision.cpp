#include "collision.h"
using namespace std;

static vector<glm::vec3> xold;
static vector<glm::vec3> xcur;
static vector<glm::vec3> xgrad;
const float miu = 1000;//罚函数惩罚系数
const float rou = 0.9992f;//切比雪夫加速收敛谱半径
float timeStep = 0.002f;//时步
long CUTPOINT = 0;
void board_phase(Collision& c, const std::vector<Mesh*> m,int num, float length) {
	//spatial hash board phase
	std::vector<Tri*> Detect_Tri;
	for (int i = 0; i < m.size();i++) {
		for (int j = 0; j < m[i]->tri.size();j++) {
			Detect_Tri.push_back(m[i]->tri[j]);
		}
	}
	//std::cout << "TriNum:" << Detect_Tri.size() << std::endl;
	c.CollisionArea = UpdateTriangle(num, Detect_Tri, length);
}
void narrow_phase(Collision& c,impactZone *ip) {
	CreateImpact(c, ip);
}
glm::vec3 position(const Vertice* v, double t) {
	return v->x0 + (float)t * (v->x - v->x0);
}
impactZone* Collision_response() {
	impactZone* zone = new impactZone();
	return zone;
}
std::vector<glm::vec3> node_positions(std::vector<Vertice*> nodes) {
	vector<glm::vec3> xs(nodes.size());
	for (int i = 0; i < xs.size();i++) {
		xs[i] = nodes[i]->x;
	}
	return xs;
}
std::vector<glm::vec3> node_grad(std::vector<Vertice*> nodes) {
	vector<glm::vec3> xs(nodes.size());
	for (int i = 0; i < xs.size(); i++) {
		xs[i] = nodes[i]->x - nodes[i]->x0;
	}
	return xs;
}
//求出当前顶点数组和起始位置的偏差值
double minGx(const std::vector<glm::vec3>& Vnext) {
	double result = 0;
	if (Vnext.size() != ::xold.size()) {
		cout << "detection error!" << endl;
		return result;
	}
	for (int i = 0; i < Vnext.size(); i++) {
		glm::vec3 temp = Vnext[i] - ::xold[i];
		result += glm::length(temp);
	}
	return result;
}
//优化目标值
double min_target(Collision& c,const std::vector<glm::vec3>& Vnext) {
	double gx = minGx(Vnext);
	double cx = 0;
	double norm2_lamda = 0;//结构和原先不同，考虑如何修改约束中对应顶点
	//std::cout << "CxLen:" << c.Cx.size() << std::endl;
	for (int i = 0; i < c.Cx.size(); i++) {
		double temp = 0;
		IneqCon* derivedObj = dynamic_cast<IneqCon*>(c.Cx[i]);
		if (derivedObj->type == 0) {
			temp = std::max(-1.0 * Cvf(derivedObj->nodes[0]->x, derivedObj->nodes[1]->x, derivedObj->nodes[2]->x, derivedObj->nodes[3]->x) + (derivedObj->lamda / miu), 0.0);
		}
		else if (derivedObj->type == 1) {
			temp = std::max(-1.0 * Cee(derivedObj->nodes[0]->x, derivedObj->nodes[1]->x, derivedObj->nodes[2]->x, derivedObj->nodes[3]->x) + (derivedObj->lamda / miu), 0.0);;
		}
		cx += temp;
		norm2_lamda += derivedObj->lamda * derivedObj->lamda;
	}
	cx = cx * cx * miu / 2;
	//std::cout << "Gx:" << gx <<std::endl;
	//std::cout << "Cx:" << cx << std::endl;
	return gx + cx - (norm2_lamda / (2.0 * miu));
}
void CreateNode(Tri* t1, Tri* t2, impact& i, impactZone* ip, Collision& c) {
	Vertice* t1_x = t1->f->ver[0];
	Vertice* t1_y = t1->f->ver[1];
	Vertice* t1_z = t1->f->ver[2];
	Vertice* t2_x = t2->f->ver[0];
	Vertice* t2_y = t2->f->ver[1];
	Vertice* t2_z = t2->f->ver[2];
	int numPri = ip->impacts.size();
	//std::cout << numPri << std::endl;
	//三角形碰撞检测，包含6组顶点和面以及9组边对的检测
	if (collision_test(0, t1_x, t2_x, t2_y, t2_z, i)) ip->impacts.push_back(i);
	if (collision_test(0, t1_y, t2_x, t2_y, t2_z, i)) ip->impacts.push_back(i);
	if (collision_test(0, t1_z, t2_x, t2_y, t2_z, i)) ip->impacts.push_back(i);
	if (collision_test(0, t2_x, t1_x, t1_y, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(0, t2_y, t1_x, t1_y, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(0, t2_z, t1_x, t1_y, t1_z, i)) ip->impacts.push_back(i);

	if (collision_test(1, t1_x, t1_y, t1_x, t1_y, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_x, t1_y, t1_x, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_x, t1_y, t1_y, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_x, t1_z, t1_x, t1_y, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_x, t1_z, t1_x, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_x, t1_z, t1_y, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_y, t1_z, t1_x, t1_y, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_y, t1_z, t1_x, t1_z, i)) ip->impacts.push_back(i);
	if (collision_test(1, t1_y, t1_z, t1_y, t1_z, i)) ip->impacts.push_back(i);

	if (ip->impacts.size() > numPri) {
		//std::cout << c.Tris.size() << std::endl;
		if (!t1->mark) {
			t1->mark = true;
			c.CollisionTri.push_back(t1);
		}
		for (int i = 0; i < t1->adj.size(); i++) {
			if (!c.Tris[t1->adj[i]]->mark) {
				c.Tris[t1->adj[i]]->mark = true;
				c.CollisionTri.push_back(c.Tris[t1->adj[i]]);
			}
		}
		if (!t2->mark) {
			t2->mark = true;
			c.CollisionTri.push_back(t2);
		}
		for (int i = 0; i < t2->adj.size(); i++) {
			if (!c.Tris[t2->adj[i]]->mark) {
				c.Tris[t2->adj[i]]->mark = true;
				c.CollisionTri.push_back(c.Tris[t2->adj[i]]);
			}
		}
	}
	//把所有碰撞的需要处理的三角形设置其对应约束，用于后续碰撞处理
	//std::cout << "CollisionNum: " << c.CollisionTri.size() << std::endl;
	SetConstraint(c);
	//std::cout << "ConstraintNum: " << c.Cx.size() << std::endl;
}
//把存在碰撞的三角面的顶点存入
void CreateNodeSeq(Collision& c, impactZone* ip) {
	std::set<Vertice*> uniqueSet;
	for (int i = 0; i < c.CollisionTri.size();i++) {
		if (c.CollisionTri[i]->obj == 1) {
			continue;
		}
		uniqueSet.insert(c.CollisionTri[i]->f->ver[0]);
		uniqueSet.insert(c.CollisionTri[i]->f->ver[1]);
		uniqueSet.insert(c.CollisionTri[i]->f->ver[2]);
	}
	for (int i = 0; i < uniqueSet.size();i++) {
		ip->nodes.assign(uniqueSet.begin(), uniqueSet.end());
	}
}
//根据boardphase产生的collisionHash来生成潜在碰撞域
void CreateImpact(Collision& c, impactZone* ip) {
	ip->impacts.clear();
	ip->nodes.clear();
	c.CollisionTri.clear();
	//std::cout <<"CollisionNum: " << c.CollisionArea.size() << std::endl;
	for (int i = 0; i < c.CollisionArea.size(); i++) {
		for (int j = 0; j < c.CollisionArea[i].cell.size() - 1; j++) {
			for (int k = j + 1; k < c.CollisionArea[i].cell.size(); k++) {
				impact temp_ip;
				CreateNode(c.CollisionArea[i].cell[j], c.CollisionArea[i].cell[k], temp_ip, ip, c);
			}
		}
	}
	CreateNodeSeq(c, ip);
	std::cout <<"VerticeNum: " << ip->nodes.size() << std::endl;
}
void SetTriConstraint(Collision& c, const Tri* x, const Tri* y) {
	SetCvf(c, x->f->ver[0], x->f->ver[1], x->f->ver[2], y->f->ver[0]);
	SetCvf(c, x->f->ver[0], x->f->ver[1], x->f->ver[2], y->f->ver[1]);
	SetCvf(c, x->f->ver[0], x->f->ver[1], x->f->ver[2], y->f->ver[2]);
	SetCvf(c, y->f->ver[0], y->f->ver[1], y->f->ver[2], x->f->ver[0]);
	SetCvf(c, y->f->ver[0], y->f->ver[1], y->f->ver[2], x->f->ver[1]);
	SetCvf(c, y->f->ver[0], y->f->ver[1], y->f->ver[2], x->f->ver[2]);

	SetCee(c, x->f->ver[0], x->f->ver[1], y->f->ver[0], y->f->ver[1]);
	SetCee(c, x->f->ver[0], x->f->ver[1], y->f->ver[0], y->f->ver[2]);
	SetCee(c, x->f->ver[0], x->f->ver[1], y->f->ver[1], y->f->ver[2]);
	SetCee(c, x->f->ver[0], x->f->ver[2], y->f->ver[0], y->f->ver[1]);
	SetCee(c, x->f->ver[0], x->f->ver[2], y->f->ver[0], y->f->ver[2]);
	SetCee(c, x->f->ver[0], x->f->ver[2], y->f->ver[1], y->f->ver[2]);
	SetCee(c, x->f->ver[1], x->f->ver[2], y->f->ver[0], y->f->ver[1]);
	SetCee(c, x->f->ver[1], x->f->ver[2], y->f->ver[0], y->f->ver[2]);
	SetCee(c, x->f->ver[1], x->f->ver[2], y->f->ver[1], y->f->ver[2]);
}
void SetCvf(Collision& c,Vertice* x, Vertice* y, Vertice* z, Vertice* p) {
	IneqCon *ic = new IneqCon();
	ic->nodes[0] = x;
	ic->nodes[1] = y;
	ic->nodes[2] = z;
	ic->nodes[3] = p;
	ic->type = 0;
	ic->lamda = 1;
	ic->mu = 0.4;
	c.Cx.push_back(ic);
}
void SetCee(Collision& c,Vertice* x1, Vertice* x2, Vertice* y1, Vertice* y2) {
	IneqCon* ic = new IneqCon();
	ic->nodes[0] = x1;
	ic->nodes[1] = x2;
	ic->nodes[2] = y1;
	ic->nodes[3] = y2;
	ic->type = 1;
	ic->lamda = 1;
	ic->mu = 0.4;
	c.Cx.push_back(ic);
}
void SetConstraint(Collision& c) {
	//std::cout << c.CollisionTri.size() << std::endl;
	c.Cx.clear();
	for (int i = 0; i < c.CollisionTri.size();i++) {
		for (int j = i + 1; j < c.CollisionTri.size();j++) {
			SetTriConstraint(c, c.CollisionTri[i], c.CollisionTri[j]);
		}
	}
}
bool collision_test(int type, const Vertice* n0, const Vertice* n1, const Vertice* n2, const Vertice* n3, impact& ip) {
	ip.type = type;
	ip.node[0] = (Vertice*)n0;
	ip.node[1] = (Vertice*)n1;
	ip.node[2] = (Vertice*)n2;
	ip.node[3] = (Vertice*)n3;
	const glm::vec3& x0 = n0->x0;
	const glm::vec3& v0 = n0->x - x0;
	glm::vec3 x1 = n1->x0 - x0, x2 = n2->x0 - x0, x3 = n3->x0 - x0;
	glm::vec3 v1 = (n1->x - n1->x0) - v0;
	glm::vec3 v2 = (n2->x - n2->x0) - v0;
	glm::vec3 v3 = (n3->x - n3->x0) - v0;
	double a = stp(v1, v2, v3);
	double b = stp(x1, v2, v3) + stp(v1, x2, v3) + stp(v1, v2, x3);
	double c = stp(v1, x2, x3) + stp(x1, v2, x3) + stp(x1, x2, v3);
	double d = stp(x1, x2, x3);
	double t[4];//存放解
	int solve_num = solve_cubic(a, b, c, d, t);
	t[solve_num] = 1; //check at end of timestep
	for (int i = 0; i < solve_num; i++) {
		if (t[i] < 0 || t[i] > 1)continue;
		ip.t = t[i];
		glm::vec3 x0 = position(n0, t[i]);
		glm::vec3 x1 = position(n1, t[i]);
		glm::vec3 x2 = position(n2, t[i]);
		glm::vec3 x3 = position(n3, t[i]);
		glm::vec3& n = ip.n;
		double* w = ip.w;
		double d;
		bool inside;
		if (type == 0) {
			// point to face collision
			d = VF_distance(x0, x1, x2, x3, &n, w);
			inside = (min(-w[1], min(-w[2], -w[3])) >= -1e-6);
		}
		else {
			// edge to edge collision
			d = EE_distance(x0, x1, x2, x3, &n, w);
			inside = (min(min(-w[0], -w[1]), min(-w[2], -w[3])) >= -1e-6);
		}
		if (glm::dot(n, (float)w[1] * v1 + (float)w[2] * v2 + (float)w[3] * v3) > 0) {
			n = -n;
		}
		if (abs(d) < 1e-6 && inside) {
			return true;
		}
	}
	return false;
}
struct NormalOpt : public NLConOpt {
	impactZone* zone;
	double inv_m;
	std::vector<double> lagrangian;
	NormalOpt() : zone(NULL), inv_m(0) { nvar = ncon = 0; }
	NormalOpt(impactZone* zone) : zone(zone), inv_m(0) {
		nvar = zone->nodes.size() * 3;
		ncon = zone->impacts.size();
		for (int n = 0; n < zone->nodes.size(); n++)
			inv_m += zone->nodes[n]->mass_inv;
		inv_m /= zone->nodes.size();
		for (int n = 0; n < ncon; n++) {
			lagrangian.push_back(0.0);
		}
	}
	void initialize(double* x) const;
	void precompute(const double* x) const;
	double objective(const double* x) const;
	void obj_grad(const double* x, double* grad) const;
	void next_step(double* x, const double* grad, double alpha);
	double constraint(const double* x, int j, int& sign) const;
	double Suggest_step(const double* x, const double* solve);
	double Fx_value(const double* x, const double* pre);
	void con_grad(const double* x, int i, double factor, double* grad) const;
	void update_lagrangian();
	void finalize(const double* x) const;
	void updateVelocity(double alpha, const double* grad);
};
void NormalOpt::update_lagrangian() {
	for (int i = 0; i < lagrangian.size();i++) {
		const impact& impact = zone->impacts[i];
		double Cx = 0;
		if (impact.type == 0) {
			Cx = std::max(-1.0 * Cvf(impact.node[0]->x, impact.node[1]->x, impact.node[2]->x, impact.node[3]->x), 0.0);
		}
		else if (impact.type == 1) {
			Cx = std::max(-1.0 * Cee(impact.node[0]->x, impact.node[1]->x, impact.node[2]->x, impact.node[3]->x), 0.0);
		}
		lagrangian[i] += miu * Cx;
	}
}
//把impactZone中的所有节点位置序列化输入一个x数组
void NormalOpt::initialize(double* x) const {
	for (int n = 0; n < zone->nodes.size(); n++)
		set_subvec(x, n, zone->nodes[n]->x);
}
//将优化迭代器中的结果返回给impactZone中的所有节点(传指针)
void NormalOpt::precompute(const double* x) const {
	for (int n = 0; n < zone->nodes.size(); n++) {
		zone->nodes[n]->x = get_subvec(x, n);
	}
}
//把impactZone中的所有节点梯度序列化输入一个x数组
void NormalOpt::obj_grad(const double* x, double* grad) const {
	for (int n = 0; n < zone->nodes.size(); n++) {
		const Vertice* node = zone->nodes[n];
		//glm::vec3 dx = (node->x - node->x0) / 0.002f;
		glm::vec3 dx = (node->x - node->x0);
		set_subvec(grad, n, dx);
	}
}
void NormalOpt::next_step(double* x,const double* grad,double alpha) {
	for (int i = 0; i < nvar;i++) {
		x[i] = x[i] - grad[i] * alpha;
	}
}
double NormalOpt::Fx_value(const double* x, const double* pre) {
	double Cx = 0;
	double lamda_normal2 = 0;
	int sign = 0;
	for (int i = 0; i < lagrangian.size();i++) {
		lamda_normal2 += lagrangian[i] * lagrangian[i];
	}
	for (int j = 0; j < ncon;j++) {
		Cx += constraint(x,j,sign);
	}
	double Gx = target(x, pre, nvar);
	//std::cout << "Gx:" << Gx << std::endl;
	//std::cout << "Cx:" << Cx << std::endl;
	return Gx + miu / 2 * Cx * Cx - (lamda_normal2 / (2 * miu));
}
double NormalOpt::Suggest_step(const double* x,const double* solve) {
	double* node = new double[nvar];
	std::copy(x, x + nvar, node);
	double alpha = timeStep;//初始化步长
	double gamma = 0.5;//wolfe条件判断准则
	double Fx = Fx_value(node,x);//初始函数值
	double nextFx = 1000;
	double normalGrad = normal_grad(solve,nvar);
	std::cout << "Fx:" << Fx << std::endl;
	//std::cout << "Comp:" << normalGrad << std::endl;
	while (nextFx >= (Fx - gamma * alpha * normalGrad * normalGrad) && alpha > 1e-9) {
		std::copy(x, x + nvar, node);
		next_step(node,solve,alpha);
		precompute(node);//前进一步后的顶点位置
		nextFx = Fx_value(node, x);
		if (nextFx >= Fx) {
			alpha *= 0.1;
			continue;
		}
		//std::cout << "nextFx:" << nextFx << std::endl;
		alpha *= 0.8;
	}
	precompute(x);
	return alpha;
}
double NormalOpt::objective(const double* x) const {
	double e = 0;
	for (int n = 0; n < zone->nodes.size(); n++) {
		const Vertice* node = zone->nodes[n];
		glm::vec3 dx = node->x - node->x0;
		e += inv_m / node->mass_inv * glm::dot(dx,dx) / 2;
	}
	return e;
}
void NormalOpt::finalize(const double* x) const {
	precompute(x);
}
//计算impact的约束值
double NormalOpt::constraint(const double* x, int j, int& sign) const {
	sign = 1;
	double temp = 0;
	//type=0为VF碰撞，type=1为EE碰撞
		const impact& impact = zone->impacts[j];
		if (impact.type == 0) {
			temp += std::max(-1.0 * Cvf(impact.node[0]->x, impact.node[1]->x, impact.node[2]->x, impact.node[3]->x) + (lagrangian[j] / miu), 0.0);
		}
		else if (impact.type == 1) {
			temp += std::max(-1.0 * Cee(impact.node[0]->x, impact.node[1]->x, impact.node[2]->x, impact.node[3]->x) + (lagrangian[j] / miu), 0.0);
		}
	return temp;
}
void NormalOpt::con_grad(const double* x, int j, double factor,double* grad) const {
	const impact& impact = zone->impacts[j];
	for (int n = 0; n < 4; n++) {
		int i = find(impact.node[n], zone->nodes);
		if (i != -1)
			add_subvec(grad, i, (float)(factor * impact.w[n]) * impact.n);
	}
}
void NormalOpt::updateVelocity(double alpha, const double* grad) {
	double inv_time = 1.0 / alpha;
	//std::cout << "Inv_time:" << inv_time << std::endl;
	double* velocity = new double[nvar];
	for (int n = 0; n < nvar; n++) {
		velocity[n] = inv_time * grad[n];
	}
	for (int n = 0; n < zone->nodes.size(); n++)
		zone->nodes[n]->v += get_subvec(velocity, n);
}
void augmented_lagrangian(NormalOpt& n) {
	//std::cout << "Num:" << n.nvar << std::endl;
	double* Op_Pre = new double[n.nvar];//上一时刻位置
	double* Op_X = new double[n.nvar];//这一时刻位置
	double* Op_grad = new double[n.nvar];
	n.initialize(Op_Pre);
	n.initialize(Op_X);
	n.obj_grad(Op_X,Op_grad);
	double alpha = 0;
	int k = 0;
	float w = 1;
	double Fx;//初始目标函数值
	double Fx2 = 0;//迭代目标函数值
	Fx = n.Fx_value(Op_Pre, Op_X);
	for (int j = 0; j < 100; j++) {
		alpha = n.Suggest_step(Op_X, Op_grad);
		//alpha = 0.002;
		if (k < 10)w = 1;
		else if (k == 10) {
			w = 2 / (2 - rou * rou);
		}
		else {
			w = 4 / (4 - rou * rou * w);
		}
		k++;
		for (int e = 0; e < n.nvar; e++) {
			double temp = Op_X[e];
			Op_X[e] -= Op_grad[e] * alpha * w;//向负梯度方向移动
			Op_grad[e] = (Op_X[e] - temp);
			//Op_grad[e] = (Op_X[e] - Op_Pre[e]);
		}
		n.updateVelocity(alpha, Op_grad);
		n.update_lagrangian();
		Fx2 = n.Fx_value(Op_Pre, Op_X);
		if (abs(Fx - Fx2) < 1e-9 || Fx2 < 1e-6) {
			std::cout << "Stop!" << std::endl;
			break;
		}
		else {
			Fx = Fx2;
		}
		//CUTPOINT++;
	}
	for (int i = 0; i < n.zone->nodes.size(); i++) {
		n.zone->nodes[i]->x0 = n.zone->nodes[i]->x;
	}
	n.precompute(Op_X);
}
void Next_step(std::vector<glm::vec3>& n, const std::vector<glm::vec3>& grad, double alpha) {
	if (n.size() != grad.size())return;
	else {
		for (int i = 0; i < n.size(); i++) {
			n[i] -= grad[i] * (float)alpha;
		}
	}
}
double Gradient_decent(Collision &c, int index) {
	vector<glm::vec3> pre = node_positions(c.ip[index]->nodes);
	vector<glm::vec3> cur = node_positions(c.ip[index]->nodes);
	vector<glm::vec3> grad = node_grad(c.ip[index]->nodes);
	double alpha = timeStep;
	double gamma = 0.5;//wolfe条件判断准则
	double Fx = min_target(c, pre);
	double nextFx = min_target(c, cur);
	double normalGrad = Vector_norm(grad);
	std::cout << "A GD Step!" << std::endl;
	//std::cout << "Comp:" << normalGrad << std::endl;
	while (nextFx >= (Fx - gamma * alpha * normalGrad * normalGrad) && alpha > 1e-6) {
		//std::cout << "Fx:" << Fx << std::endl;
		//std::cout << "Comp:" << normalGrad << std::endl;
		cur = pre;
		Next_step(cur, grad, alpha);
		//std::cout << "nextFx1:" << nextFx << std::endl;
		for (int i = 0; i < c.ip[index]->nodes.size();i++) {
			//showVec3(grad[i]);
			c.ip[index]->nodes[i]->x -= (float)alpha * grad[i];
		}
		//std::cout << "Comp:" << normalGrad << std::endl;
		nextFx = min_target(c, cur);
		//std::cout << "nextFx:" << nextFx << std::endl;
		for (int i = 0; i < c.ip[index]->nodes.size(); i++) {
			c.ip[index]->nodes[i]->x += (float)alpha * grad[i];
		}
		alpha *= 0.6;
	}
	return alpha;
}
void Collision_Solve(Collision& c) {
	for (int i = 0; i < c.ip.size(); i++) {
		if (c.ip[i]->nodes.size() < 1)continue;
		double alpha = 0;
		//NormalOpt* nlp = new NormalOpt(c.ip[i]);
		::xold = node_positions(c.ip[i]->nodes);//存放最初所有impact顶点的位置拷贝
		double Fx;//初始目标函数值
		double Fx2;//迭代值
		Fx = min_target(c, ::xold);
		//std::cout << Fx << std::endl;
		int k = 0;
		float w = 1;
		std::cout << "A CS function!Nodes Num:" << c.ip[i]->nodes.size() << std::endl;
		::xgrad = node_grad(c.ip[i]->nodes);
		for (int j = 0; j < 100; j++) {
			alpha = timeStep;
			//alpha = Gradient_decent(c,i);//得到一个合适的步长;
			//std::cout << "Alpha："<< alpha << std::endl;
			if (k < 10)w = 1;
			else if (k == 10) {
				w = 2 / (2 - rou * rou);
			}
			else {
				w = 4 / (4 - rou * rou * w);
			}
			//修改nodes中顶点位置
			k++;
			for (int n = 0; n < c.ip[i]->nodes.size(); n++) {
				c.ip[i]->nodes[n]->x0 = c.ip[i]->nodes[n]->x;
				c.ip[i]->nodes[n]->x -= (xgrad[n] * (float)alpha) * w;
			}
			//对每个约束更新拉格朗日乘子
			for (int n = 0; n < c.Cx.size(); n++) {
				double temp = 0;
				IneqCon* derivedObj = dynamic_cast<IneqCon*>(c.Cx[n]);
				if (derivedObj->type == 0) {
					temp = std::max(-1.0 * Cvf(derivedObj->nodes[0]->x, derivedObj->nodes[1]->x, derivedObj->nodes[2]->x, derivedObj->nodes[3]->x) + (derivedObj->lamda / miu), 0.0);
				}
				else if (derivedObj->type == 1) {
					temp = std::max(-1.0 * Cee(derivedObj->nodes[0]->x, derivedObj->nodes[1]->x, derivedObj->nodes[2]->x, derivedObj->nodes[3]->x) + (derivedObj->lamda / miu), 0.0);
				}
				derivedObj->lamda += miu * temp;
			}
			::xcur = node_positions(c.ip[i]->nodes);
		    Fx2 = min_target(c, ::xcur);
			//残差足够小时结束迭代
			std::cout << "Fx:" << Fx << std::endl;
			std::cout << "Fx2:" << Fx2 << std::endl;
			if (abs(Fx - Fx2) < 1e-12) {
				std::cout << "Stop!" << std::endl;
				break;
			}
			else {
				Fx = Fx2;
			}
		}
	}
}
void Collision_Resolve(Collision& c) {
	if (CUTPOINT >= infs) {
		std::cout << "Runtime Error"<< std::endl;
		return;
	}
	for (int i = 0; i < c.ip.size(); i++) {
		if (c.ip[i]->nodes.size() < 1)continue;
		augmented_lagrangian(NormalOpt(c.ip[i]));
	}
}