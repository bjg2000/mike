#include "darttrajcalc.h"

void dartdrag(traj s, double* dx, double a, double g)
{
    dx[0] = s.dx;
    dx[1] = s.dy;
    dx[2] = s.dz;
    dx[3] = -a * s.dx * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz);
    dx[4] = -a * s.dy * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz) - g;
    dx[5] = -a * s.dz * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz);
}

void rk4_blaster(traj* s, double a, double g)
{

    for (int i = 0; i < (gridsize - 1); i++)
    {
        double k1[6], k2[6], k3[6], k4[6];
        double dt = (s + i + 1)->t - (s + i)->t;

        traj sk1 = *(s + i), sk2 = *(s + i), sk3 = *(s + i);

        dartdrag(*(s + i), k1, a, g);   //k1
        sk1.x = (s + i)->x + 0.5 * dt * k1[0];
        sk1.y = (s + i)->y + 0.5 * dt * k1[1];
        sk1.z = (s + i)->z + 0.5 * dt * k1[2];
        sk1.dx = (s + i)->dx + 0.5 * dt * k1[3];
        sk1.dy = (s + i)->dy + 0.5 * dt * k1[4];
        sk1.dz = (s + i)->dz + 0.5 * dt * k1[5];

        dartdrag(sk1, k2, a, g);   //k2
        sk2.x = (s + i)->x + 0.5 * dt * k2[0];
        sk2.y = (s + i)->y + 0.5 * dt * k2[1];
        sk2.z = (s + i)->z + 0.5 * dt * k2[2];
        sk2.dx = (s + i)->dx + 0.5 * dt * k2[3];
        sk2.dy = (s + i)->dy + 0.5 * dt * k2[4];
        sk2.dz = (s + i)->dz + 0.5 * dt * k2[5];

        dartdrag(sk2, k3, a, g);   //k3
        sk3.x = (s + i)->x + dt * k3[0];
        sk3.y = (s + i)->y + dt * k3[1];
        sk3.z = (s + i)->z + dt * k3[2];
        sk3.dx = (s + i)->dx + dt * k3[3];
        sk3.dy = (s + i)->dy + dt * k3[4];
        sk3.dz = (s + i)->dz + dt * k3[5];

        dartdrag(sk3, k4, a, g);   //k4
        (s + i + 1)->x = (s + i)->x + (dt / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
        (s + i + 1)->y = (s + i)->y + (dt / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
        (s + i + 1)->z = (s + i)->z + (dt / 6) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
        (s + i + 1)->dx = (s + i)->dx + (dt / 6) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
        (s + i + 1)->dy = (s + i)->dy + (dt / 6) * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
        (s + i + 1)->dz = (s + i)->dz + (dt / 6) * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]);
    }
}

void linspace(traj* arr, double n)
{
    for (int i = 0; i < gridsize; i++)
    {
        arr[i].t = i * n / (double)(gridsize - 1);
    }
}

unsigned int tol_check(double value_actual, double value_theoretical, double tol)
{
    double lower_bound = value_theoretical - tol;
    double upper_bound = value_theoretical + tol;

    if ((value_actual < lower_bound) || (value_actual > upper_bound))
        return 0;
    else
        return 1;
}

unsigned int tol_pos(traj target, traj* calculated_hit, double tol)
{
    double t_hit = calculated_hit[gridsize - 1].t;
    double x_hit = calculated_hit[gridsize - 1].x;
    double y_hit = calculated_hit[gridsize - 1].y;
    double z_hit = calculated_hit[gridsize - 1].z;

    double x_target_t = target.x + target.dx * t_hit;
    double y_target_t = target.y + target.dy * t_hit;
    double z_target_t = target.z + target.dz * t_hit;

    unsigned int x_in_bounds = tol_check(x_hit, x_target_t, tol);
    unsigned int y_in_bounds = tol_check(y_hit, y_target_t, tol);
    unsigned int z_in_bounds = tol_check(z_hit, z_target_t, tol);

    if (x_in_bounds && y_in_bounds && z_in_bounds)
        return 1;
    else
        return 0;
}

//objective function
double myfunc(unsigned n, const double* x, double* grad, void* my_func_data)
{
    return x[2];    //return t
}

//nonlinear constraint function
void myconstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* data)
{
    my_constraint_data* d = (my_constraint_data*)data;

    //unpack decision variables
    double v0 = d->a;
    double alpha = d->b;
    traj target_init = d->c;
    traj dartpath[gridsize];

    dartpath[0].x = 0;
    dartpath[0].y = 0;
    dartpath[0].z = 0;
    dartpath[0].dx = v0 * cos(x[0]) * cos(x[1]);
    dartpath[0].dy = v0 * sin(x[0]) * cos(x[1]);
    dartpath[0].dz = v0 * sin(x[1]);

    double T = x[2];

    linspace(dartpath, T);

    rk4_blaster(dartpath, alpha, 9.81);

    result[0] = dartpath[gridsize - 1].x - (target_init.x + target_init.dx * T);
    result[1] = dartpath[gridsize - 1].y - (target_init.y + target_init.dy * T);
    result[2] = dartpath[gridsize - 1].z - (target_init.z + target_init.dz * T);
}

traj_calc_output traj_calc(traj target, double v_init, double rho, double c_d, double A, double m)
{
    double lb[3] = { -(90 * PI / 180) , -(90 * PI / 180), 0 }; // lower bounds 
    double ub[3] = { (90 * PI / 180) , (90 * PI / 180), 5 }; // upper bounds 
    nlopt_opt opt;

    opt = nlopt_create(NLOPT_LN_COBYLA, 3); // algorithm and dimensionality 
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_min_objective(opt, myfunc, NULL);
    nlopt_set_maxeval(opt, 100);

    //elevation angle guess
    double guessphi = 0 * PI / 180;

    //lead angle guess
    double guesstheta = 0 * PI / 180;

    //time to target guess
    double guesst = 1;

    //setup output data struct
    traj_calc_output outputdata;
    outputdata.phi = guessphi;
    outputdata.theta = guesstheta;
    outputdata.theta = guesst;
    outputdata.success = 0;

    double alpha = 0.5 * rho * c_d * A / m;
    my_constraint_data data[3] = { v_init, alpha , target };

    double tol[3] = { 1e-8, 1e-8, 1e-8 };
    nlopt_add_equality_mconstraint(opt, 3, myconstraint, &data, tol);

    nlopt_set_xtol_rel(opt, 1e-8);

    double x[3] = { guessphi, guesstheta, guesst };  // `*`some` `initial` `guess`*` 
    double minf; // `*`the` `minimum` `objective` `value,` `upon` `return`*` 

    int result = nlopt_optimize(opt, x, &minf);

    if (result > 0)
    {
        traj target_hit[gridsize];
        target_hit[0].x = 0;
        target_hit[0].y = 0;
        target_hit[0].z = 0;
        target_hit[0].dx = v_init * cos(x[0]) * cos(x[1]);
        target_hit[0].dy = v_init * sin(x[0]) * cos(x[1]);
        target_hit[0].dz = v_init * sin(x[1]);
        linspace(target_hit, x[2]);
        rk4_blaster(target_hit, alpha, 9.81);

        //make sure converged solution hits target within 3 inch radius
        if (tol_pos(target, target_hit, 0.25 / 3.28))
        {
            outputdata.phi = x[0];
            outputdata.theta = x[1];
            outputdata.timetotarget = x[2];
            outputdata.success = 1;
        }
    }

    nlopt_destroy(opt);
    return outputdata;
}