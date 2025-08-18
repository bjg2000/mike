#include <stdio.h>
#include <math.h>
#include <nlopt.h>
#include <time.h>

#define gridsize 100
#define PI 3.141592

struct traj {
    double t;
    double x;
    double y;
    double z;
    double dx;
    double dy;
    double dz;
};

typedef struct {
    double a, b;
    traj c;
} my_constraint_data;

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
    return x[2];    //return t
}

//constraint function
void myconstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* data)
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

int main() {

    clock_t begin = clock();

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

    //init params
    double v_init = 300/3.28;
    double alpha = 0.044305;
    traj inittarget;
    inittarget.x = 170 / 3.28;
    inittarget.y = 0 / 3.28;
    inittarget.z = 0 / 3.28;
    inittarget.dx = 0 / 3.28;
    inittarget.dy = 0 / 3.28;
    inittarget.dz = 0 / 3.28;

    my_constraint_data data[3] = { v_init, 0.044305 , inittarget };

    double tol[3] = {1e-8, 1e-8, 1e-8};
    nlopt_add_equality_mconstraint(opt, 3, myconstraint, &data, tol);

    nlopt_set_xtol_rel(opt, 1e-8);

    double x[3] = { guessphi, guesstheta, guesst };  // `*`some` `initial` `guess`*` 
    double minf; // `*`the` `minimum` `objective` `value,` `upon` `return`*` 

    
    int result = nlopt_optimize(opt, x, &minf);
    //printf("%d\n", result);
    //printf("%d\n", nlopt_get_numevals(opt));


    if ((result != 6) && (result > 0)) {
        //printf("found minimum at f(%g,%g,%g) = %0.10g\n", x[0], x[1], x[2], minf);
        //printf("\r\n");
        traj target_hit[gridsize];
        target_hit[0].x = 0;
        target_hit[0].y = 0;
        target_hit[0].z = 0;
        target_hit[0].dx = v_init * cos(x[0]) * cos(x[1]);
        target_hit[0].dy = v_init * sin(x[0]) * cos(x[1]);
        target_hit[0].dz = v_init * sin(x[1]);

        linspace(target_hit, x[2]);
        rk4_blaster(target_hit, alpha, 9.81);

        printf("target hit at(%g, %g, %g) feet\n", target_hit[gridsize - 1].x * 3.28, target_hit[gridsize - 1].y * 3.28, target_hit[gridsize - 1].z * 3.28);
        printf("with elevation %g degrees, and lead %g degrees\n", x[0] * 180 / PI, x[1] * 180 / PI);
        printf("time to target: %g seconds\n", x[2]);
    }
    else {
        //printf("\r\n");
        printf("nlopt failed!\n");
    }

    clock_t end = clock();
    printf("time to calculate solution: %lf ms\n", (double)(end - begin) / CLOCKS_PER_SEC * 1000);
    nlopt_destroy(opt);

    return 0;
}