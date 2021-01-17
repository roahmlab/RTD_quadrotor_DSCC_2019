# Quadrotor RTD

## About

This repository includes the MATLAB code used to simulate quadrotors and perform Reachability-based Trajectory Design (RTD) for our ASME DSCC 2019 paper [1], which won the best student paper prize.

RTD is a method for safe online trajectory planning for mobile robots. Check out the [RTD repo](https://github.com/skvaskov/RTD) (for car-like robots) and the [RTD tutorial](https://github.com/skousik/RTD_tutorial).

## Dependencies

You'll need the following repositories on your MATLAB path:

1. [simulator](https://github.com/skousik/simulator)
2. [CORA 2018](https://tumcps.github.io/CORA/) **NOTE this code will NOT work with CORA 2020**

## Getting Started

Run the script `run_quadrotor_simulation_static_obstacles.m` in the `simulation` folder.

Planning with RTD happens in the `quadrotor_RTD_zono_planner.m` class, which gets called by the `simulator` (from the repo of the same name).

If you want to walk through the reachable set computation, start at `step_1_frs_computation`; note that computing the tracking error tables in step 2 can take several hours, so we have provided some precomputed tracking error tables for you.

### Authors

Shreyas Kousik, Patrick Holmes, and Zehui Lu

### References

[1] Kousik, S., Holmes, P. and Vasudevan, R., 2019, October. Safe, aggressive quadrotor flight via reachability-based trajectory design. In *Dynamic Systems and Control Conference* (Vol. 59162, p. V003T19A010). American Society of Mechanical Engineers. [link](https://asmedigitalcollection.asme.org/DSCC/proceedings/DSCC2019/59162/V003T19A010/1070634?casa_token=ZejWuBeKAngAAAAA:gKMET5yu5PNsDJ1Q69Nl41Rnn3VhAvqNqZPqCsS9aq0PfmOkSL9O7bbnIT260P6DbWxzKA)

[2] Kousik, S., Holmes, P. and Vasudevan, R., 2019. Technical Report: Safe, Aggressive Quadrotor Flight via Reachability-based Trajectory Design. *arXiv preprint arXiv:1904.05728*. [link](https://arxiv.org/abs/1904.05728)