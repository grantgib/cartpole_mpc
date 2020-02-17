%% MPC Cartpole Regulator MPC (pinned model)
clear; clc; close all;

%% Path setup
restoredefaultpath;
if isunix
%     addpath('../../casadi-linux-matlabR2014b-v3.5.1');
    import casadi.*
else
    addpath('../Toolboxes/casadi-windows-matlabR2014b-3.5.1');
    import casadi.*
end
addpath(genpath('utils/'));

%% Choose initial condition and reference state
ex = 1;
if ex == 1
    % example 1 conditions
    x_init = [0 ; pi; 0; 0];        % initial condition.
    x_ref = [2 ; pi; 0; 0];         % Reference posture.
    u_ref = 0;
elseif ex == 2
    % example 2 conditions
    x_init = [0 ; pi-0.25; 0; 0];         % Reference posture.
    x_ref = [0 ; pi; 0; 0];       % initial condition.'
    u_ref = 0;
end

%% Generate Cartpole Dynamics
% Assumed pinned dynamics
[f_nonlinear,f_linear,E_nonlinear,H_nonlinear,E_linear,H_linear,DDQ_REF,n_x, n_c, length_pole] = ...
    Generate_Cartpole_Dynamics();
disp("Finished computing dynamics!");

%% Build Nonlinear program
DT = 0.005;         % Control update (sec)
N = 200;            % Prediction Horizon
type_reg = true;    % Regulator problem
[solver_NL,solver_L] = ...
    Formulate_NLP(DT,N,n_x,n_c,f_nonlinear,f_linear,E_nonlinear,E_linear,H_nonlinear,H_linear,DDQ_REF,type_reg);
disp("Finished formulating NLP!");

%% Simulation Time
sim_time = 1;      % Maximum simulation time (sec)

%% *********************************************************
%       Run Nonlinear Regulator Simulation
%***********************************************************
disp("Begin Simulation...");
[x_traj_NL,u_traj_NL,x_traj_all_NL,t_all_NL,mpciter_NL,args_NL] = ...
    Simulate_NMPC(x_init,DT,N,n_x,n_c,f_nonlinear,...
                  solver_NL,sim_time,type_reg,x_ref,u_ref);
disp("Finished Simulation!");

% Save Trajectory
save_traj = 0;
if save_traj
    X_REF = x_traj_NL; 
    U_REF = u_traj_NL;
    if ex == 1
        file_name = "cartpole_reference_nonlinear_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    elseif ex == 2
        file_name = "cartpole_reference_nonlinear_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

% Plot
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all_NL,x_traj_NL,length_pole,animate_traj,type_reg,2,DT,x_ref);
Plot_Cartpole(t_all_NL,x_traj_NL,u_traj_NL,sim_time,args_NL,plot_traj,mpciter_NL,x_ref);
sgtitle('Nonlinear Regulator');

%% Omit linear reference trajectories for now (not really useful since we get nonlinear trajectories from FROST for biped walkers)
% *********************************************************
%       Run Linear Regulator Simulation
%***********************************************************



% *********************************************************
%       Linear vs. Nonlinear Comparison
%***********************************************************




