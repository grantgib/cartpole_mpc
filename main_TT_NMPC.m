%% MPC Cartpole Trajectory Tracking Nonlinear MPC
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
type_reg = false;   % Trajectory Tracking problem
[solver, args] = Formulate_NLP(DT,N,n_x,n_c,f_nonlinear,type_reg);
disp("Finished formulating NLP!");

%% Load Reference Trajectory
ex1 = false;
if ex1
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_trajectory_ex1.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
else
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_trajectory_ex2.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
end

%% Run simulation
% Choose initial condition and reference state
if ex1
    % example 1 conditions
    x_init = [0 ; pi; 0; 0] +...
             [0.2; 0.1; 0.05; 0.05];      % initial condition.
else
    % example 2 conditions
    x_init = [0.5 ; pi+0.4; 0.2; -0.1] +...
             [-0.1; -0.1; -0.05; -0.05];        % Reference posture.
end
% Simulate
sim_time = 10;      % Maximum simulation time (sec)
disp("Begin Simulation...");
[x_traj,u_traj,x_traj_all,t_all,mpciter] = Simulate_NMPC(x_init, DT, N, n_x, n_c,...
                                                 f_nonlinear, solver, args,...
                                                 sim_time,type_reg,X_REF,U_REF);
disp("Finished Simulation!");

%% Save Trajectory
save_traj = true;
if save_traj
    if ex1
        file_name = "cartpole_trajtrack_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    else
        file_name = "cartpole_trajtrack_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

%% Plot
close all;
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all,x_traj,length_pole,animate_traj,X_REF);
Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,X_REF_Original,U_REF_Original);







