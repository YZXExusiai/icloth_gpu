#include "constraint.h"
using namespace std;

double EqCon::value(int* sign) {
    if (sign) *sign = 0;
    return dot(n, node->x - x);
}
MeshGrad EqCon::gradient() { MeshGrad grad; grad[node] = n; return grad; }
MeshGrad EqCon::project() { return MeshGrad(); }
double EqCon::energy(double value) { return stiff * sq(value) / 2.; }
double EqCon::energy_grad(double value) { return stiff * value; }
double EqCon::energy_hess(double value) { return stiff; }
MeshGrad EqCon::friction(double dt, MeshHess& jac) { return MeshGrad(); }

double IneqCon::value(int* sign) {
    if (sign)
        *sign = 1;
    double d = 0;
    for (int i = 0; i < 4; i++)
        d += w[i] * dot(n, nodes[i]->x);
    //d -= ::magic.repulsion_thickness;
    return d;
}

MeshGrad IneqCon::gradient() {
    MeshGrad grad;
    for (int i = 0; i < 4; i++)
        grad[nodes[i]] = (float)w[i] * n;
    return grad;
}

MeshGrad IneqCon::project() {
    double d = value();
    if (d >= 0)
        return MeshGrad();
    double inv_mass = 0;
    for (int i = 0; i < 4; i++)
        if (free[i])
            inv_mass += sq(w[i]) * nodes[i]->mass_inv;
    MeshGrad dx;
    for (int i = 0; i < 4; i++)
        if (free[i])
            dx[nodes[i]] = (float)(- (w[i] * nodes[i]->mass_inv) / inv_mass)  * n * (float)d;
    return dx;
}

double violation(double value) { return std::max(-value, 0.); }

double IneqCon::energy(double value) {
    double v = violation(value);
    return stiff * v * v * v / 6;
}
double IneqCon::energy_grad(double value) {
    return -stiff * sq(violation(value))  / 2;
}
double IneqCon::energy_hess(double value) {
    return stiff * violation(value);
}

MeshGrad IneqCon::friction(double dt, MeshHess& jac) {
    if (mu == 0)
        return MeshGrad();
    double fn = abs(energy_grad(value()));
    if (fn == 0)
        return MeshGrad();
    glm::vec3 v = glm::vec3(0);
    double inv_mass = 0;
    for (int i = 0; i < 4; i++) {
        v += (float)w[i] * nodes[i]->v;
        if (free[i])
            inv_mass += sq(w[i]) * nodes[i]->mass_inv;
    }
    glm::mat3x3 T = glm::mat3x3(1) - glm::outerProduct(n, n);
    double vt = glm::length(T * v);
    double f_by_v = min(mu * fn / vt, 1 / (dt * inv_mass));
    // double f_by_v = mu*fn/max(vt, 1e-1);
    MeshGrad force;
    for (int i = 0; i < 4; i++) {
        if (free[i]) {
            force[nodes[i]] = (float)(- w[i] * f_by_v)* T* v;
            for (int j = 0; j < 4; j++) {
                if (free[j]) {
                    jac[make_pair(nodes[i], nodes[j])] = (float)(- w[i] * w[j] * f_by_v)* T;
                }
            }
        }
    }
    return force;
}