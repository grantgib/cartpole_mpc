%% Load_Saved_Trajectory
clear; clc; close all
%% Path setup
if isunix
    %     addpath('../../casadi-linux-matlabR2014b-v3.5.1');
    import casadi.*
else
    addpath('Toolboxes/casadi-windows-matlabR2014b-3.5.1');
    import casadi.*
end
addpath(genpath('utils/'));
addpath(genpath('Saved_Trajectories'));

%% Choose Reference Trajectory
ex = 2;

%% Nonlinear Load
type_reg = false;  % true for reference traj

if type_reg
    name = "reference_nonlinear";
else
    name = "TT_nonlinear";
end

if ex == 1
    ex = "_ex1";
elseif ex == 2
    ex = "_ex2";
end

loadname = "cartpole_"+name+ex+".mat";
load(loadname);

% Animate and Plot
if type_reg
    % Regulator Problem
    animate_traj = true;
    plot_traj = true;
    Animate_Cartpole(t_all_NL,x_traj_NL,length_pole,animate_traj,type_reg,2,DT,x_ref);
    Plot_Cartpole(t_all_NL,x_traj_NL,u_traj_NL,sim_time,args_NL,plot_traj,mpciter_NL,x_ref);
    sgtitle("Nonlinear Regulator");
    
else
    % Trajectory Tracking Problem
    animate_traj = true;
    plot_traj = true;
    anim_time = 2;
    Animate_Cartpole(t_all_NL,x_traj_NL,length_pole,animate_traj,type_reg,2,DT,X_REF_Original);
    Plot_Cartpole(t_all_NL,x_traj_NL,u_traj_NL,sim_time,args_NL,plot_traj,mpciter_NL,X_REF_Original,U_REF_Original);
    sgtitle("Nonlinear Trajectory Tracking");
end

%% Linear Load
type_reg = false;  % true trajectories don't exist yet

if type_reg
    name = "reference_linear";
else
    name = "TT_linear";
end

if ex == 1
    ex = "_ex1";
elseif ex == 2
    ex = "_ex2";
end

loadname = "cartpole_"+name+ex+".mat";
load(loadname);

% Animate and Plot
if type_reg
    % Regulator Problem
    animate_traj = true;
    plot_traj = true;
    Animate_Cartpole(t_all_L,x_traj_L,length_pole,animate_traj,type_reg,2,DT,x_ref);
    Plot_Cartpole(t_all_L,x_traj_L,u_traj_L,sim_time,args_NL,plot_traj,mpciter_L,x_ref);
    sgtitle("Linear Regulator");
else
    % Trajectory Tracking Problem
    animate_traj = true;
    plot_traj = true;
    anim_time = 2;
    Animate_Cartpole(t_all_L,x_traj_L,length_pole,animate_traj,type_reg,anim_time,DT,X_REF_Original);
    Plot_Cartpole(t_all_L,x_traj_L,u_traj_L,sim_time,args_NL,plot_traj,mpciter_L,X_REF_Original,U_REF_Original);
    sgtitle("Linear Trajectory Tracking");
end

%% Compare
% Calculate Errors
error_L = x_traj_L - X_REF_Original(:,1:size(x_traj_L,2));
error_NL = x_traj_NL - X_REF_Original(:,1:size(x_traj_L,2));
plot_compare = true;
plot_time = 2;
Plot_Cartpole_TrajectoryComparison(t_all_L,x_traj_L,u_traj_L,error_L,args_L,mpciter_L,...
    t_all_NL,x_traj_NL,u_traj_NL,error_NL,args_NL,mpciter_NL,...
    X_REF_Original,U_REF_Original,...
    plot_time,plot_compare);








