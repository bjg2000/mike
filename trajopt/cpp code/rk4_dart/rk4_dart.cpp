// rk4_blaster_r3.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cmath>
#include <array>

#define gridsize 100
#define PI 3.141592

void linspace(float* t, float n);

struct traj {
    float t[gridsize];
    float x[gridsize];
    float y[gridsize];
    float z[gridsize];
    float dx[gridsize];
    float dy[gridsize];
    float dz[gridsize];
};

void main()
{
    double m = 13.0 / 1000.0;
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



}

void dartdrag(float* x, float* dx, float a, float g)
{
    dx[0] = x[3];
    dx[1] = x[4];
    dx[2] = x[5];
    dx[3] = -a * x[3] * sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
    dx[4] = -a * x[4] * sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) - g;
    dx[5] = -a * x[5] * sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
}

void rk4_blaster(traj* s, float s0, float a, float g)
{
    for (int i = 0; i < (gridsize - 1); i++)
    {
        float k1[6], k2[6], k3[6], k4[6];
        float dt = s->t[i + 1] - s->t[i];

        dartdrag();
        //k1
        //2*k2
        //2*k3
        //k4

    }
}

void linspace(float* arr, float n)
{
    for (int i = 0; i < gridsize; i++)
    {
        arr[i] = i * n / (float)(gridsize - 1);
    }
}
