#include <stdio.h>
#include <math.h>
#include <time.h>
#include "darttrajcalc.h"

void main()
{
    //init params (these are passed to the function from the main program)
    //------------------------------------------------------------------------------------------------------------------------------
    double v_init = 190 / 3.28, rho = 1.293, c_d = 0.6712, A = PI * (13.0 / 1000.0 / 2.0) * (13.0 / 1000.0 / 2.0), m = 1.3 / 1000;
    traj target;
    target.x = 100 / 3.28;
    target.y = 0 / 3.28;
    target.z = 0 / 3.28;
    target.dx = 0 / 3.28;
    target.dy = 0 / 3.28;
    target.dz = 10 / 3.28;
    //------------------------------------------------------------------------------------------------------------------------------

    clock_t begin = clock();

    traj_calc_output dart_hit = traj_calc(target, v_init, rho, c_d, A, m);;

    /*
    traj target_hit[gridsize];
    target_hit[0].x = 0;
    target_hit[0].y = 0;
    target_hit[0].z = 0;
    target_hit[0].dx = v_init * cos(outputdata.phi) * cos(outputdata.theta);
    target_hit[0].dy = v_init * sin(outputdata.phi) * cos(outputdata.theta);
    target_hit[0].dz = v_init * sin(outputdata.theta);
    linspace(target_hit, outputdata.timetotarget);
    rk4_blaster(target_hit, 0.5 * rho * c_d * A / m, 9.81);
    printf("target hit at (%.2f, %.2f, %.2f) feet\n", target_hit[gridsize - 1].x * 3.28, target_hit[gridsize - 1].y * 3.28, target_hit[gridsize - 1].z * 3.28);
    */

    if (dart_hit.success)
    {
        printf("elevation %.2f degrees and lead %.2f degrees\n", dart_hit.phi * 180 / PI, dart_hit.theta * 180 / PI);
        printf("time to target: %.2f seconds\n", dart_hit.timetotarget);
    }
    else
    {
        printf("Target out of range!\n");
    }

    clock_t end = clock();
    printf("calculation time: %f ms\n", (double)(end - begin) / CLOCKS_PER_SEC * 1000);
}



