#include"geometry.h"
glm::vec3 FaceNormal(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
	glm::vec3 ab = b - a;
	glm::vec3 ac = c - a;
	return glm::cross(ab, ac);
}
glm::vec3 Projection(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x) {
	glm::vec3 n = FaceNormal(a, b, c);
	float px = (n.x * n.y * a.y + n.y * n.y * x.x - n.x * n.y * x.y + n.x * n.z * a.z + n.z * n.z * x.x - n.x * n.z * x.z + n.x * n.x * a.x) / (n.x * n.x + n.y * n.y + n.z * n.z);
	float py = (n.y * n.z * a.z + n.z * n.z * x.y - n.y * n.z * x.z + n.y * n.x * a.x + n.x * n.x * x.y - n.x * n.y * x.x + n.y * n.y * a.y) / (n.x * n.x + n.y * n.y + n.z * n.z);
	float pz = (n.x * a.x * n.z + n.x * n.x * x.z - n.x * x.x * n.z + n.y * a.y * n.z + n.y * n.y * x.z - n.y * x.y * n.z + n.z * n.z * a.z) / (n.x * n.x + n.y * n.y + n.z * n.z);
	return glm::vec3(px, py, pz);
}
glm::vec3 Barycentric(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x) {
	float i = (-(x.x - b.x) * (c.y - b.y) + (x.y - b.y) * (c.x - b.x)) / (-(a.x - b.x) * (c.y - b.y) + (a.y - b.y) * (c.x - b.x));
	float j = (-(x.x - c.x) * (a.y - c.y) + (x.y - b.y) * (a.x - c.x)) / (-(b.x - c.x) * (a.y - c.y) + (b.y - c.y) * (a.x - c.x));
	float k = 1.0f - i - j;
	return glm::vec3(i, j, k);
}
double VF_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& x, glm::vec3* n, double* w) {
	glm::vec3 temp_n;
	if (!n)n = &temp_n;
	double temp_w[4];
	if (!w)w = temp_w;
	*n = glm::cross(glm::normalize(b - a), glm::normalize(c - a));
	if (glm::dot(*n, *n) < 1e-6) {
		//三个点在一条线上不构成平面
		return infinity;
	}
	*n = glm::normalize(*n);
	double h = glm::dot(x - a, *n);//x点到平面距离
	double b0 = stp(b - x, c - x, *n);
	double b1 = stp(c - x, a - x, *n);
	double b2 = stp(a - x, b - x, *n);
	w[0] = 1;
	w[1] = -b0 / (b0 + b1 + b2);
	w[2] = -b1 / (b0 + b1 + b2);
	w[3] = -b2 / (b0 + b1 + b2);
	return h;
}
double EE_distance(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& y0, const glm::vec3& y1, glm::vec3* n, double* w) {
	glm::vec3 temp_n;
	if (!n)n = &temp_n;
	double temp_w[4];
	if (!w)w = temp_w;
	*n = glm::cross(glm::normalize(x1 - x0), glm::normalize(y1 - y0));
	if (glm::dot(*n, *n) < 1e-6) {
		return infinity;
	}
	*n = glm::normalize(*n);
	double h = glm::dot(x0 - y0, *n);
	double a0 = stp(y1 - x1, y0 - x1, *n);
	double a1 = stp(y0 - x0, y1 - x0, *n);
	double b0 = stp(x0 - y1, x1 - y1, *n);
	double b1 = stp(x1 - y0, x0 - y0, *n);
	w[0] = a0 / (a0 + a1);
	w[1] = -a1 / (a0 + a1);
	w[2] = -b0 / (b0 + b1);
	w[3] = -b1 / (b0 + b1);
	return h;
}
int sgn(double x) {
	if (x > 0)return 1;
	else if (x < 0)return -1;
	else {
		return 0;
	}
}
double stp(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
	return glm::dot(a, glm::cross(b, c));
}
int solve_quadratic(double a, double b, double c, double x[2]) {
	double d = b * b - 4 * a * c;
	if (d < 0) {
		x[0] = -b / (2 * a);//如果没有解，就取其极值点
		return 0;
	}
	double q = -(b + sgn(b) * sqrt(d)) / 2;
	int i = 0;
	if (abs(a) > 1e-12 * abs(q)) {
		x[i++] = q / a;
	}
	if (abs(q) > 1e-12 * abs(c)) {
		x[i++] = c / q;//韦达定理
	}
	if (i == 2 && x[0] > x[1]) {
		std::swap(x[0], x[1]);//数值大的解放在前面
	}
	return i;
}
double newtons_method(double a, double b, double c, double d, double x0, int init_dir) {
	if (init_dir != 0) {
		//使用2阶泰勒展开获得X0处的近似，并假设一阶微分y' = 0
		double y0 = d + x0 * (c + x0 * (b + x0 * a));//在x0处方程值
		double ddy0 = 2 * b + 6 * a * x0;//二阶导
		x0 += init_dir * sqrt(abs(2 * y0 / ddy0));
	}
	for (int iter = 0; iter < 100; iter++) {
		double y = d + x0 * (c + x0 * (b + x0 * a));
		double dy = 3 * a * x0 * x0 + 2 * b * x0 + c;
		if (dy == 0) {
			return x0;
		}
		double x1 = x0 - y / dy;
		if (abs(x0 - x1) < 1e-6) {
			return x0;
		}
		x0 = x1;
	}
	return x0;
}
int solve_cubic(double a, double b, double c, double d, double x[3]) {
	double xc[2];//存放极值点
	int ncrit = solve_quadratic(3 * a, 2 * b, c, xc);//一阶导数计算极值点个数
	if (ncrit == 0) {
		x[0] = newtons_method(a, b, c, d, xc[0], 0);
		return 1;
	}
	else if (ncrit == 1) {
		return solve_quadratic(b, c, d, x);//只有一个极值点时退化为2次方程求解
	}
	else {
		double yc[2];//存放两个极值点对应的y值
		yc[0] = d + xc[0] * (c + xc[0] * (b + xc[0] * a));
		yc[1] = d + xc[1] * (c + xc[1] * (b + xc[1] * a));
		int i = 0;
		if (yc[0] * a >= 0) {
			x[i++] = newtons_method(a, b, c, d, xc[0], -1);
		}
		//两个极值点异号，说明中间有解,选择更接近0的遍历
		if (yc[0] * yc[1] <= 0) {
			int closer = abs(yc[0]) < abs(yc[1]) ? 0 : 1;
			x[i++] = newtons_method(a, b, c, d, xc[closer], closer == 0 ? 1 : -1);
		}
		if (yc[1] * a <= 0) {
			x[i++] = newtons_method(a, b, c, d, xc[1], 1);
		}
		return i;
	}
}
float Cvf(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x) {
	glm::vec3 n = glm::normalize(FaceNormal(a, b, c));
	if (glm::length(n) <= 1e-6)return 0;
	glm::vec3 p = x - a;
	return glm::dot(n, p);
}
float Cee(glm::vec3 x1, glm::vec3 x2, glm::vec3 x3, glm::vec3 x4) {
	glm::vec3 e1 = x2 - x1;
	glm::vec3 e2 = x4 - x3;
	glm::vec3 n = glm::cross(e1, e2);
	if (glm::length(n) <= 1e-6)return 0;//两边平行，法向量为0向量
	glm::vec3 t1 = glm::dot(glm::cross(n, e2), e1) * e1 + x1;
	glm::vec3 t2 = glm::dot(glm::cross(n, e1), e2) * e2 + x3;
	glm::vec3 t = t2 - t1;
	return glm::dot(n, t);
}
double Vector_norm(const std::vector<glm::vec3>& V) {
	double value = 0;
	for (int i = 0; i < V.size(); i++) {
		value += glm::length(V[i]);
	}
	return value;
}
void showVec3(const glm::vec3& v) {
	std::cout << "X: " << v.x << " Y: " << v.y << " Z: " << v.z << std::endl;
}
template <typename T> T clamp(const T& x, const T& a, const T& b) {
	return std::min(std::max(x, a), b);
}