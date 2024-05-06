#include "optimization.h"
double target(const double* x, const double* x_old, int size) {
    double temp = 0;
    double len = 0;
    for (int i = 0; i < size / 3; i++) {
        len = 0;
        len += (x[3 * i] - x_old[3 * i]) * (x[3 * i] - x_old[3 * i]);
        len += (x[3 * i + 1] - x_old[3 * i + 1]) * (x[3 * i + 1] - x_old[3 * i + 1]);
        len += (x[3 * i + 2] - x_old[3 * i + 2]) * (x[3 * i + 2] - x_old[3 * i + 2]);
        temp += sqrt(len);
    }
    return temp / size * 3;
}
double normal_grad(const double* x, int size) {
    double temp = 0;
    for (int i = 0; i < size; i++) {
        temp += x[i]* x[i];
    }
    return temp;
}
void augmented_lagrangian_method(const NLOpt& problem) {

}