#pragma once
#include <vector>
#include <glm\glm.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include "sparse.h"
// Problems
static long infs = 11451400;
struct NLOpt { 
    // nonlinear optimization problem
    // minimize objective
    int nvar;
    virtual void initialize(double* x) const = 0;
    virtual double objective(const double* x) const = 0;
    virtual void precompute(const double* x) const {}
    virtual void gradient(const double* x, double* g) const = 0;
    virtual bool hessian(const double* x, SpMat<double>& H) const {
        return false; // should return true if implemented
    };
    virtual void finalize(const double* x) const = 0;
};

struct NLConOpt { 
    // nonlinear constrained optimization problem
    // minimize objective s.t. constraints = or <= 0
    int nvar, ncon;
    virtual void initialize(double* x) const = 0;
    virtual void precompute(const double* x) const {}
    virtual double objective(const double* x) const = 0;
    virtual void obj_grad(const double* x, double* grad) const = 0; // set
    virtual double constraint(const double* x, int j, int& sign) const = 0;
    virtual void con_grad(const double* x, int j, double factor,
        double* grad) const = 0; // add factor*gradient
    virtual void finalize(const double* x) const = 0;
};
double target(const double* x, const double* x_old, int size);
double normal_grad(const double* x,int size);
void augmented_lagrangian_method(const NLOpt& problem, bool verbose = false);
inline glm::vec3 get_subvec(const double* x, int i) {
    return glm::vec3(x[i * 3 + 0], x[i * 3 + 1], x[i * 3 + 2]);
}
inline void set_subvec(double* x, int i, const glm::vec3& xi) {
    for (int j = 0; j < 3; j++) x[i * 3 + j] = xi[j];
}
inline void add_subvec(double* x, int i, const glm::vec3& xi) {
    for (int j = 0; j < 3; j++) x[i * 3 + j] += xi[j];
}