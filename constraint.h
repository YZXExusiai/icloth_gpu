#pragma once
#include "mesh.h"
#include <vector>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <map>
#include "geometry.h"

typedef std::map<Vertice*, glm::vec3> MeshGrad;
typedef std::map<std::pair<Vertice*, Vertice*>,glm::mat3x3> MeshHess;

struct Constraint {
    virtual ~Constraint() {};
    virtual double value(int* sign = NULL) = 0;
    virtual MeshGrad gradient() = 0;
    virtual MeshGrad project() = 0;
    // energy function
    virtual double energy(double value) = 0;
    virtual double energy_grad(double value) = 0;
    virtual double energy_hess(double value) = 0;
    // frictional force
    virtual MeshGrad friction(double dt, MeshHess& jac) = 0;
};
//��ʽԼ��
struct EqCon : public Constraint {
    Vertice* node;
    glm::vec3 x, n;
    double stiff;
    double value(int* sign = NULL);
    MeshGrad gradient();
    MeshGrad project();
    double energy(double value);
    double energy_grad(double value);
    double energy_hess(double value);
    MeshGrad friction(double dt, MeshHess& jac);
};
//����ʽԼ��
struct IneqCon : public Constraint {
    Vertice* nodes[4];
    double w[4];
    bool free[4];
    int type;//type=0ΪVF��ײ��type=1ΪEE��ײ��VF�����0,1,2�±�Ϊface,3Ϊvertice
    glm::vec3 n; // normal
    double a; // area
    double mu; // friction
    double lamda; // Lagrange ratio
    double stiff;
    double value(int* sign = NULL);
    MeshGrad gradient();
    MeshGrad project();
    double energy(double value);
    double energy_grad(double value);
    double energy_hess(double value);
    MeshGrad friction(double dt, MeshHess& jac);
};