%% MPC Cartpole Trajectory Tracking MPC (pinned)
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
addpath(genpath('Saved_Trajectories'));

%% Load Reference Trajectory and choose initial conidtion
ex = 2;
if ex == 1
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_nonlinear_ex1.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
    % example 1 conditions
    x_init_ref = [0; pi; 0; 0] % initial condition from reference
    delta_x_init = [0.1; 0.05; .01; 0.02]
    x_init = x_init_ref + delta_x_init     % initial condition.
elseif ex == 2
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_nonlinear_ex2.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
    % example 2 conditions
    x_init_ref = [0 ; pi-0.25; 0; 0]
    delta_x_init = [-0.075; -0.1; -0.05; -0.05]
    x_init = x_init_ref + delta_x_init        % Reference posture.
end

%% Generate Cartpole Dynamics
% Assume pinned model
disp("Begin Dynamics Computation...");
[f_nonlinear,f_linear,E_nonlinear,H_nonlinear,E_linear,H_linear,n_x, n_u, l_pole] = ...
    Generate_Cartpole_Dynamics();
disp("Finished computing dynamics!");

%% Build Nonlinear program
DT = 0.005;         % Control update (sec)
N = 200;            % Prediction Horizon
type_reg = false;   % Trajectory Tracking problem
disp("Begin NLP formulation...");
[solver_NL,solver_L] = ...
    Formulate_NLP(DT,N,n_x,n_u,f_nonlinear,f_linear,E_nonlinear,E_linear,H_nonlinear,H_linear,type_reg);
disp("Finished formulating NLP!");

%% Simulation time
sim_time = 1;      % Maximum simulation time (sec)

%% *********************************************************
%       Run Nonlinear Trajectory Tracking Simulation
%***********************************************************
disp("Begin Nonlinear Simulation...");
[x_traj_NL,u_traj_NL,x_traj_all_NL,t_all_NL,mpciter_NL,args_NL] = ...
    Simulate_NMPC(x_init, DT, N, n_x, n_u,f_nonlinear,solver_NL,...
    sim_time,type_reg,X_REF_Original,U_REF_Original);
disp("Finished Nonlinear Simulation!");

% Save Trajectory
save_traj = 1;
if save_traj
    if ex == 1
        file_name = "cartpole_TT_nonlinear_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    elseif ex == 2
        file_name = "cartpole_TT_nonlinear_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    end
end

% Plot
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all_NL,x_traj_NL,l_pole,animate_traj,type_reg,1,DT,X_REF);
Plot_Cartpole(t_all_NL,x_traj_NL,u_traj_NL,sim_time,args_NL,plot_traj,mpciter_NL,X_REF_Original,U_REF_Original);
sgtitle('Nonlinear Trajectory Tracking');


%% *********************************************************
%       Run Linear Trajectory Tracking Simulation
%***********************************************************
% pause

% Simulate
disp("Begin Linear Simulation...");
[x_traj_L,u_traj_L,dx_traj_L,du_traj_L,dx_traj_all_L,t_all_L,mpciter_L,args_L] = ...
    Simulate_LMPC(delta_x_init,DT,N,n_x,n_u,f_linear,f_nonlinear,solver_L,...
                  sim_time,type_reg,X_REF_Original,U_REF_Original);
disp("Finished Linear Simulation!");

% Save Trajectory
save_traj = 1;
if save_traj
    if ex == 1
        file_name = "cartpole_TT_linear_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    elseif ex == 2
        file_name = "cartpole_TT_linear_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    end
end
% Plot
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all_L,x_traj_L,l_pole,animate_traj,type_reg,1,DT,X_REF);
Plot_Cartpole(t_all_L,x_traj_L,u_traj_L,sim_time,args_NL,plot_traj,mpciter_L,X_REF_Original,U_REF_Original);
sgtitle('Linear Trajectory Tracking');

%% *********************************************************
%       Linear vs. Nonlinear Comparison
%***********************************************************
% Calculate Errors
error_L = x_traj_L - X_REF_Original(:,1:size(x_traj_L,2));
error_NL = x_traj_NL - X_REF_Original(:,1:size(x_traj_L,2));
plot_compare = true;
plot_time = 2;
Plot_Cartpole_TrajectoryComparison(t_all_L,x_traj_L,u_traj_L,error_L,args_L,mpciter_L,...
    t_all_NL,x_traj_NL,u_traj_NL,error_NL,args_NL,mpciter_NL,...
    X_REF_Original,U_REF_Original,...
    plot_time,plot_compare);


















