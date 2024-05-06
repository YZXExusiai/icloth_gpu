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
double VF_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& x, glm::vec3* n, double* w);//���ص������
double EE_distance(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& y0, const glm::vec3& y1, glm::vec3* n, double* w);//���ر߱߾���
int sgn(double x);
glm::vec3 FaceNormal(glm::vec3 a, glm::vec3 b, glm::vec3 c);
glm::vec3 Projection(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
glm::vec3 Barycentric(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
double stp(glm::vec3 a, glm::vec3 b, glm::vec3 c);//�������������Ļ�ϻ�
int solve_quadratic(double a, double b, double c, double x[2]);//����һԪ���η��̵Ľ⣬���ؽ������������д��x����
double newtons_method(double a, double b, double c, double d, double x0, int init_dir);//ţ�ٵ��������һԪ���η���,����x0�ǽ�,init_dirΪ��ʼ��������
int solve_cubic(double a, double b, double c, double d, double x[3]);//����һԪ���η��̵ĽⲢд��,����ccd���
template <typename T> T clamp(const T& x, const T& a, const T& b);//�бƺ���������[a��b]�����ֵ
double VF_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& x, glm::vec3* n, double* w);//���ص�����벢�޸�
double EE_distance(const glm::vec3& x0, const glm::vec3& x1, const glm::vec3& y0, const glm::vec3& y1, glm::vec3* n, double* w);//���ر߱߾��벢�޸�
float Cvf(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 x);
float Cee(glm::vec3 x1, glm::vec3 x2, glm::vec3 x3, glm::vec3 x4);
double Vector_norm(const std::vector<glm::vec3>& V);
void showVec3(const glm::vec3 &v);
