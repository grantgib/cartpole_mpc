%% MPC Cartpole Trajectory Tracking Linear MPC
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
pinned = true;
linearize = true;
analytical = false;
[f_linear, f_nonlinear,n_x, n_c, length_pole] = ...
    Generate_Cartpole_Dynamics(pinned,analytical);
disp("Finished computing dynamics!");

%% Build Nonlinear program
DT = 0.005;         % Control update (sec)
N = 200;            % Prediction Horizon
type_reg = false;   % Trajectory Tracking problem
[solver, args] = Formulate_NLP(DT,N,n_x,n_c,f_linear,type_reg,linearize);
disp("Finished formulating NLP!");

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

%% Run simulation
% Choose initial condition and reference state
if ex1
    % example 1 conditions
    x_init_ref = [0 ; pi; 0; 0] % initial conditionfrom reference
    dx_init = [0; 0.1; 0; 0] % initial delta state
%     dx_init = zeros(4,1);
else
    % example 2 conditions
    x_init_ref = [0.5 ; pi+0.4; 0.2; -0.1]
    dx_init = [-0.1; -0.1; -0.05; -0.05]        
end
% Simulate
sim_time = 5;      % Maximum simulation time (sec)
disp("Begin Simulation...");
[x_traj,u_traj,dx_traj,du_traj,dx_traj_all,t_all,mpciter] = ...
    Simulate_LMPC(dx_init,DT,N,n_x,n_c,f_linear,f_nonlinear,solver,args,...
                  sim_time,type_reg,X_REF_Original,U_REF_Original);
disp("Finished Simulation!");

%% Save Trajectory
save_traj = true;
if save_traj
    if ex1
        file_name = "cartpole_trajtrack_lin_ex1";
        save(fullfile(pwd, 'Saved_Trajectories', file_name));
    else
        file_name = "cartpole_trajtrack_lin_ex2";
        save(fullfile(pwd, 'Saved_Trajectories', file_name)); 
    end
end

%% Plot
animate_traj = true;
plot_traj = true;
Animate_Cartpole(t_all,x_traj,length_pole,animate_traj,X_REF);
Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,X_REF_Original,U_REF_Original);



















