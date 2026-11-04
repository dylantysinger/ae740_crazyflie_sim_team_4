/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_ocp_pendulum_ode_a89c0237.h"

#define NX     OCP_PENDULUM_ODE_A89C0237_NX
#define NZ     OCP_PENDULUM_ODE_A89C0237_NZ
#define NU     OCP_PENDULUM_ODE_A89C0237_NU
#define NP     OCP_PENDULUM_ODE_A89C0237_NP


int main()
{
    int status = 0;
    ocp_pendulum_ode_a89c0237_sim_solver_capsule *capsule = ocp_pendulum_ode_a89c0237_acados_sim_solver_create_capsule();
    status = ocp_pendulum_ode_a89c0237_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = ocp_pendulum_ode_a89c0237_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = ocp_pendulum_ode_a89c0237_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = ocp_pendulum_ode_a89c0237_acados_get_sim_out(capsule);
    void *acados_sim_dims = ocp_pendulum_ode_a89c0237_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 3.141592653589793;
    x_current[2] = 0;
    x_current[3] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = ocp_pendulum_ode_a89c0237_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = ocp_pendulum_ode_a89c0237_acados_sim_free(capsule);
    if (status) {
        printf("ocp_pendulum_ode_a89c0237_acados_sim_free() returned status %d. \n", status);
    }

    ocp_pendulum_ode_a89c0237_acados_sim_solver_free_capsule(capsule);

    return status;
}
