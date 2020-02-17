# MPC_Cartpole

This repository formulates NMPC and LMPC controllers for the Cartpole. Both the regulator ('R_MPC_pinned.m') and trajectory tracking ('TT_MPC_pinned.m') problems are solved.

Previous solutions are located in the 'Saved_Trajectories' directory and animations/plots can be used to view the data using the 'Load_Saved_Trajectory.m' Script.

A multi-shooting method is used to solve the nonlinear program where the decision variables are composed of both control inputs and states. The dynamics are imposed via equality constraints. A discrete forward euler method is used to update the dynamics for each time step. The propogation can be performed using the standard system (xdot = f(x)) or the descriptor system (E(x,u) * xdot = H(x,u)). The descriptor system version is set as the default (both give equivalent results).

TODO: The linear regulator problem is not solved but can be easily formulated using linearization code from 'TT_MPC_pinned.m' as reference.
