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

//objective function
double myfunc(unsigned n, const double* x, double* grad, void* my_func_data)
{
    my_constraint_data* d = (my_constraint_data*)my_func_data;

    //unpack decision variables
    double v0 = d->a;
    double alpha = d->b;
    traj target_init = d->c;
    traj dartpath[gridsize];
    traj shot_guess;

    //calculate trajectory guess
    dartpath[0].x = 0;
    dartpath[0].y = 0;
    dartpath[0].z = 0;
    dartpath[0].dx = v0 * cos(x[0]) * cos(x[1]);
    dartpath[0].dy = v0 * sin(x[0]) * cos(x[1]);
    dartpath[0].dz = v0 * sin(x[1]);

    double T = x[2];

    linspace(dartpath, T);

    rk4_blaster(dartpath, alpha, 9.81);
    d->d = dartpath;

    //shot_guess.x = dartpath[gridsize - 1].x - (target_init.x + target_init.dx * T);
    shot_guess.y = dartpath[gridsize - 1].y - (target_init.y + target_init.dy * T);
    shot_guess.z = dartpath[gridsize - 1].z - (target_init.z + target_init.dz * T);

    double obj_output_val = T + pow(shot_guess.y, 2) + pow(shot_guess.z, 2); //minimize this
    //printf("objective function value: %0.6f\n", obj_output_val);

    return obj_output_val;    
}

//nonlinear constraint function
double myxconstraint(unsigned n, const double* x, double* grad, void* data)
{
    my_constraint_data* d = (my_constraint_data*)data;

    //unpack decision variables
    double T = x[2];

    double v0 = d->a;
    double alpha = d->b;
    traj target_init = d->c;
    traj* dartpath = d->d;
    
    return dartpath[gridsize - 1].x - (target_init.x + target_init.dx * T);
}

traj_calc_output traj_calc(traj target, double v_init, double rho, double c_d, double A, double m)
{
    double lb[3] = { -(90 * PI / 180) , -(90 * PI / 180), 0 }; // lower bounds 
    double ub[3] = { (90 * PI / 180) , (90 * PI / 180), 5 }; // upper bounds 

    nlopt_opt opt;

    double alpha = 0.5 * rho * c_d * A / m;
    my_constraint_data data[3] = { v_init, alpha , target };

    opt = nlopt_create(NLOPT_LN_COBYLA, 3); // algorithm and dimensionality 
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_min_objective(opt, myfunc, &data);
    nlopt_set_maxeval(opt, 1000);

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

    //double tol[3] = { 1e-8, 1e-8, 1e-8 };

    nlopt_add_equality_constraint(opt, myxconstraint, &data, 1e-8);

    //nlopt_set_xtol_rel(opt, 1e-8);

    double x[3] = { guessphi, guesstheta, guesst };  // `*`some` `initial` `guess`*` 
    double minf; // `*`the` `minimum` `objective` `value,` `upon` `return`*` 

    int result = nlopt_optimize(opt, x, &minf);

    //double dart_end_phi = x[0];
    //double dart_end_theta = x[1];

    //round phi and theta to the nearest 0.1 degrees
    double dart_end_phi = round(x[0] * 180 / PI * 10.0) / 10.0 * PI / 180;
    double dart_end_theta = round(x[1] * 180 / PI * 10.0) / 10.0 * PI / 180;

    //calculate final trajectory
    traj dart_end[gridsize];
    dart_end[0].x = 0;
    dart_end[0].y = 0;
    dart_end[0].z = 0;
    dart_end[0].dx = v_init * cos(dart_end_phi) * cos(dart_end_theta);
    dart_end[0].dy = v_init * sin(dart_end_phi) * cos(dart_end_theta);
    dart_end[0].dz = v_init * sin(dart_end_theta);
    linspace(dart_end, x[2]);
    rk4_blaster(dart_end, alpha, 9.81);

    outputdata.phi = dart_end_phi;
    outputdata.theta = dart_end_theta;
    outputdata.timetotarget = x[2];

    //record dart deviation from final target position
    double deviation_y = dart_end[gridsize - 1].y - (target.y + target.dy * dart_end[gridsize - 1].t);
    double deviation_z = dart_end[gridsize - 1].z - (target.z + target.dz * dart_end[gridsize - 1].t);

    double target_tol_y = 9.0 / 12.0 / 3.28;
    double target_tol_z = 6.0 / 12.0 / 3.28;

    //printf("deviation: %f, %f\n", deviation_y * 3.28 * 12, deviation_z * 3.28 * 12);
    //printf("%f\n", pow(deviation_y / target_tol_y, 2) + pow(deviation_z / target_tol_z, 2));
    //printf("%d\n", result);

    if ((result > 0) || (result == -4)) {

        //make sure converged solution hits target within specified tolerance on the y and z axes
        if ((pow(deviation_y / target_tol_y, 2) + pow(deviation_z / target_tol_z, 2)) <= 1)
        {
            outputdata.success = 1;
        }
    }

    nlopt_destroy(opt);
    return outputdata;
}