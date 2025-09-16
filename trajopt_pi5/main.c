#include <stdio.h>
//#include <math.h>
#include <time.h>
//#include "darttrajcalc.h"
#include "darttrajcalc_v3.c"

void main()
{
    //init params (these are passed to the function from the main program)
    //------------------------------------------------------------------------------------------------------------------------------
    double v_init = 200 / 3.28, rho = 1.293, c_d = 0.6712, A = PI * (13.0 / 1000.0 / 2.0) * (13.0 / 1000.0 / 2.0), m = 1.3 / 1000, angle_round = 0.3;
    traj target;
    target.x = 120 / 3.28;
    target.y = 0 / 3.28;
    target.z = 0 / 3.28;
    target.dx = 0 / 3.28;
    target.dy = 0 / 3.28;
    target.dz = 0 / 3.28;
    //------------------------------------------------------------------------------------------------------------------------------

    //algorithm hits a pole when target == 0
    if (target.x == 0) {
        target.x = 5 / 3.28;
    }
    if (target.y == 0) {
        target.y = 1e-3 / 3.28;
    }
    if (target.z == 0) {
        target.z = 1e-3 / 3.28;
    }

    clock_t begin = clock();

    traj_calc_output dart_hit = traj_calc(target, v_init, rho, c_d, A, m, angle_round);

    if (dart_hit.success)
    {
        printf("Target hit!\n");
    }
    else
    {
        printf("Target out of range!\n");
    }

    printf("elevation %.3f degrees and lead %.3f degrees\n", dart_hit.phi * 180 / PI, dart_hit.theta * 180 / PI);
    printf("time to target x position: %.2f seconds\n", dart_hit.timetotarget);

    traj dart_end[gridsize];
    dart_end[0].x = 0;
    dart_end[0].y = 0;
    dart_end[0].z = 0;
    dart_end[0].dx = v_init * cos(dart_hit.phi) * cos(dart_hit.theta);
    dart_end[0].dy = v_init * sin(dart_hit.phi) * cos(dart_hit.theta);
    dart_end[0].dz = v_init * sin(dart_hit.theta);
    linspace(dart_end, dart_hit.timetotarget);
    rk4_blaster(dart_end, 0.5 * rho * c_d * A / m, 9.81);
    printf("last dart position at (%.2f, %.2f, %.2f) feet\n", dart_end[gridsize - 1].x * 3.28, dart_end[gridsize - 1].y * 3.28, dart_end[gridsize - 1].z * 3.28);
    printf("target position at (%.2f, %.2f, %.2f) feet\n", (target.x + target.dx * dart_end[gridsize - 1].t) * 3.28, (target.y + target.dy * dart_end[gridsize - 1].t) * 3.28, (target.z + target.dz * dart_end[gridsize - 1].t) * 3.28);

    clock_t end = clock();
    printf("calculation time: %f ms\n", (double)(end - begin) / CLOCKS_PER_SEC * 1000);
}



