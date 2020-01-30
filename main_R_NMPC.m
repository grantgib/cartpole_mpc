%% MPC Cartpole Regulator Nonlinear MPC
clear; clc;

%% Path setup
if isunix
%     addpath('../../casadi-linux-matlabR2014b-v3.5.1');
    import casadi.*
else
    addpath('Toolboxes/casadi-windows-matlabR2014b-3.5.1');
    import casadi.*
end
addpath(genpath('utils/'));

%% Generate Cartpole Dynamics
[f_nonlinear, n_x, n_c, length_pole] = Generate_Cartpole_Dynamics();
disp("Finished computing dynamics!");

%% Build Nonlinear program
DT = 0.005;         % Control update (sec)
N = 200;            % Prediction Horizon
type_reg = true;    % Regulator problem
[solver, args] = Formulate_NLP(DT,N,n_x,n_c,f_nonlinear,type_reg);
disp("Finished formulating NLP!");

%% Run simulation
% Choose initial condition and reference state
ex1 = false;
if ex1
    % example 1 conditions
    x_init = [0 ; pi; 0; 0];       % initial condition.
    x_ref = [2 ; pi; 0; 0];         % Reference posture.
else
    % example 2 conditions
    x_init = [0.5 ; pi+0.4; 0.2; -0.1];         % Reference posture.
    x_ref = [0 ; pi; 0; 0];       % initial condition.
end
% Simulate
sim_time = 1;      % Maximum simulation time (sec)
disp("Begin Simulation...");
[x_traj,u_traj,x_traj_all,t_all,mpciter] = Simulate_NMPC(x_init, DT, N, n_x, n_c,...
                                                 f_nonlinear, solver, args,...
                                                 sim_time,type_reg,x_ref);
disp("Finished Simulation!");

%% Save Trajectory
save_traj = false;
if save_traj
    X_REF = x_traj;
    U_REF = u_traj;
    if ex1
        file_name = "cartpole_reference_trajectory_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    else
        file_name = "cartpole_reference_trajectory_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

%% Plot
close all;
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all,x_traj,length_pole,animate_traj);
Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,x_ref);






