// rk4_blaster_r3.c : This file contains the 'main' function. Program execution begins and ends there.
//

#include <stdio.h>
#include <math.h>

#define gridsize 100
#define PI 3.141592

typedef struct {
    double t;
    double x;
    double y;
    double z;
    double dx;
    double dy;
    double dz;
} traj;

void linspace(double* t, double n);
void rk4_blaster(traj *s, double a, double g);
void dartdrag(traj s, double* dx, double a, double g);

int main()
{
    traj dartpath[gridsize];

    double v0 = 300 /3.28;
    double phi = 2 *PI/180;
    double theta = 0 *PI/180;

    double t[gridsize];
    linspace(t, 1);

    for (int i = 0; i < gridsize; i++) {
        dartpath[i].t = t[i];
    }

    dartpath[0].x = 0;
    dartpath[0].y = 0;
    dartpath[0].z = 0;
    dartpath[0].dx = v0 * cos(phi) * cos(theta);
    dartpath[0].dy = v0 * sin(phi) * cos(theta);
    dartpath[0].dz = v0 * sin(theta);

    double m = 1.3 / 1000.0;
    double c_d = 0.6712;
    double dart_area = PI * pow(13.0 / 1000.0 / 2.0, 2);

    double alpha = dart_area * 0.5 * 1.293 * c_d / m;
    //printf("%f\n", alpha);

    /*
    traj s;
    linspace(s.t, 1);
    for (int i = 0; i < gridsize; i++) {
        printf("%f\n", s.t[i]);
    }
    */

    rk4_blaster(dartpath, alpha, 9.81);

    printf("\n");

    for (int i = 0; i < gridsize; i++) {
        printf("%f\n", dartpath[i].x*3.28);
    }

    return 0;
}

void dartdrag(traj s, double* dx, double a, double g)
{
    dx[0] = s.dx;
    dx[1] = s.dy;
    dx[2] = s.dz;
    dx[3] = -a * s.dx * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz);
    dx[4] = -a * s.dy * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz) - g;
    dx[5] = -a * s.dz * sqrt(s.dx * s.dx + s.dy * s.dy + s.dz * s.dz);
}

void rk4_blaster(traj *s, double a, double g)
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

void linspace(double* arr, double n)
{
    for (int i = 0; i < gridsize; i++)
    {
        arr[i] = i * n / (double)(gridsize - 1);
    }
}
