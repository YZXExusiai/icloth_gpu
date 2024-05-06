#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>

const double infinity = 1145141919810;

inline double sq(double x) { return x * x; }


glm::vec3 FaceNormal(glm::vec3 a, glm::vec3 b, glm::vec3 c);
double VF_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& x, glm::vec3* n, double* w);//返回点面距离
double EE_distance(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& y0, const glm::vec3& y1, glm::vec3* n, double* w);//返回边边距离
int sgn(double x);
glm::vec3 FaceNormal(glm::vec3 a, glm::vec3 b, glm::vec3 c);
glm::vec3 Projection(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
glm::vec3 Barycentric(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
double stp(glm::vec3 a, glm::vec3 b, glm::vec3 c);//返回三个向量的混合积
int solve_quadratic(double a, double b, double c, double x[2]);//计算一元二次方程的解，返回解个数，并将解写入x数组
double newtons_method(double a, double b, double c, double d, double x0, int init_dir);//牛顿迭代法求解一元三次方程,返回x0是解,init_dir为初始迭代方向
int solve_cubic(double a, double b, double c, double d, double x[3]);//计算一元三次方程的解并写入,用于ccd求解
template <typename T> T clamp(const T& x, const T& a, const T& b);//夹逼函数，返回[a，b]区间的值
double VF_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& x, glm::vec3* n, double* w);//返回点面距离并修改
double EE_distance(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& y0, const glm::vec3& y1, glm::vec3* n, double* w);//返回边边距离并修改
float Cvf(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
float Cee(glm::vec3 x1, glm::vec3 x2, glm::vec3 x3, glm::vec3 x4);
double Vector_norm(const std::vector<glm::vec3>& V);
void showVec3(const glm::vec3 &v);
