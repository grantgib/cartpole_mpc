%% MPC Cartpole Trajectory Tracking Nonlinear MPC
clear; clc; close all;

%% Path setup
if isunix
%     addpath('../../casadi-linux-matlabR2014b-v3.5.1');
    import casadi.*
else
    addpath('Toolboxes/casadi-windows-matlabR2014b-3.5.1');
    import casadi.*
end
addpath(genpath('utils/'));

%% Load Reference Trajectory
ex1 = true;
if ex1
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_trajectory_ex1.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
else
    load(fullfile(pwd,'Saved_Trajectories','cartpole_reference_trajectory_ex2.mat'),'X_REF','U_REF');
    X_REF_Original = X_REF;
    U_REF_Original = U_REF;
end

%% Generate Cartpole Dynamics
pinned = true;
[f_linear, f_nonlinear,n_x, n_c, length_pole] = ...
    Generate_Cartpole_Dynamics(pinned);
disp("Finished computing dynamics!");

%% Build Nonlinear program
DT = 0.005;         % Control update (sec)
N = 200;            % Prediction Horizon
type_reg = false;   % Trajectory Tracking problem
[solver_L,args_L,solver_NL,args_NL] = ...
    Formulate_NLP(DT,N,n_x,n_c,f_linear,f_nonlinear,type_reg,X_REF_Original,U_REF_Original);
disp("Finished formulating NLP!");

%% Choose Initial Condition + Simulation Time
if ex1
    % example 1 conditions
    x_init_ref = [0; pi; 0; 0] % initial condition from reference
    delta_x_init = [0.02; 0; 0; 0]
    x_init = x_init_ref + delta_x_init     % initial condition.
else
    % example 2 conditions
    x_init_ref = [0.5 ; pi+0.4; 0.2; -0.1]
    delta_x_init = [-0.1; -0.1; -0.05; -0.05]
    x_init = x_init_ref + delta_x_init        % Reference posture.
end
sim_time = 5;      % Maximum simulation time (sec)

%% *********************************************************
%       Run Linear Trajectory Tracking Simulation
%***********************************************************
% pause

% Simulate
disp("Begin Linear Simulation...");
[x_traj_L,u_traj_L,dx_traj_L,du_traj_L,dx_traj_all_L,t_all_L,mpciter_L] = ...
    Simulate_LMPC(delta_x_init,DT,N,n_x,n_c,f_linear,f_nonlinear,solver_L,args_L,...
                  sim_time,type_reg,X_REF_Original,U_REF_Original);
disp("Finished Linear Simulation!");

% Save Trajectory
save_traj = false;
if save_traj
    if ex1
        file_name = "cartpole_trajtrack_lin_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    else
        file_name = "cartpole_trajtrack_lin_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

% Plot
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all_L,x_traj_L,length_pole,animate_traj,X_REF);
Plot_Cartpole(t_all_L,x_traj_L,u_traj_L,sim_time,args_NL,plot_traj,mpciter_L,X_REF_Original,U_REF_Original);
sgtitle('Linear');
pause
%% *********************************************************
%       Run Nonlinear Trajectory Tracking Simulation
%***********************************************************
disp("Begin Nonlinear Simulation...");
[x_traj_NL,u_traj_NL,x_traj_all_NL,t_all_NL,mpciter_NL] = ...
    Simulate_NMPC(x_init, DT, N, n_x, n_c,f_nonlinear, solver_NL, args_NL,...
                  sim_time,type_reg,X_REF_Original,U_REF_Original);
disp("Finished Nonlinear Simulation!");

% Save Trajectory
save_traj = false;
if save_traj
    if ex1
        file_name = "cartpole_trajtrack_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    else
        file_name = "cartpole_trajtrack_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

% Plot
animate_traj = false;
plot_traj = true;
Animate_Cartpole(t_all_NL,x_traj_NL,length_pole,animate_traj,X_REF);
Plot_Cartpole(t_all_NL,x_traj_NL,u_traj_NL,sim_time,args_NL,plot_traj,mpciter_NL,X_REF_Original,U_REF_Original);
sgtitle('Nonlinear');
pause
%% *********************************************************
%       NMPC vs LMPC Comparison
%***********************************************************
close all

% Calculate Errors
error_L = x_traj_L - X_REF_Original(:,1:size(x_traj_L,2));
error_NL = x_traj_NL - X_REF_Original(:,1:size(x_traj_L,2));
plot_compare = true;
plot_time = 2;
Plot_Cartpole_TrajectoryComparison(t_all_L,x_traj_L,u_traj_L,error_L,args_L,mpciter_L,...
                                   t_all_NL,x_traj_NL,u_traj_NL,error_NL,args_NL,mpciter_NL,...
                                   X_REF_Original,U_REF_Original,...
                                   plot_time,plot_compare);
                         
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
                               
