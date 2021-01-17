## Online Planning 
This folder contains all the files needed to actually run RTD online -- but, the wrapper script to run simulations is in [another folder](https://github.com/roahmlab/RTD_quadrotor_DSCC_2019/tree/main/simulation).

The most important file in this folder is `planners/quadrotor_zono_RTD_planner.m`, which is a class implementing RTD as a trajectory planner. In particular, check out the [replan method](https://github.com/roahmlab/RTD_quadrotor_DSCC_2019/blob/main/step_3_online_planning/planners/quadrotor_zono_RTD_planner.m#L157) to see how RTD works in the loop.
