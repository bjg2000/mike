#include <math.h>
#include <nlopt.h>
#include <stdio.h>

#define gridsize 100
#define PI 3.14159265358979323846

typedef struct {
    double t;
    double x;
    double y;
    double z;
    double dx;
    double dy;
    double dz;
} traj;

typedef struct {
    double a, b;
    traj c;
    traj* d;
} my_constraint_data;

typedef struct {
    double phi, theta, timetotarget;
    unsigned int success;
} traj_calc_output;

void dartdrag(traj s, double* dx, double a, double g);
void rk4_blaster(traj* s, double a, double g);
void linspace(traj* arr, double n);
double myfunc(unsigned n, const double* x, double* grad, void* my_func_data);
void myconstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* data);
traj_calc_output traj_calc(traj target, double v_init, double rho, double c_d, double A, double m);

double myxconstraint(unsigned n, const double* x, double* grad, void* data);
double myphiconstraint(unsigned n, const double* x, double* grad, void* data);
double mythetaconstraint(unsigned n, const double* x, double* grad, void* data);