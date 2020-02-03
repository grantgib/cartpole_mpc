%% Load_Saved_Trajectory
clear;
clc;
close all;
type_reg = true;
ex1 = true;

if type_reg
    name = "reference_trajectory";
else
    name = "trajtrack";
end

if ex1
    ex = "_ex1";
else
    ex = "_ex2";
end

loadname = "cartpole_"+name+ex+".mat";
load(loadname);

%% Animate and Plot
type_reg = true;

if type_reg
    %% Regulator Problem
    animate_traj = false;
    plot_traj = true;
    Animate_Cartpole(t_all,x_traj,length_pole,animate_traj);
    Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,0,x_ref);
    
else
    %% Trajectory Tracking Problem
    animate_traj = false;
    plot_traj = true;
    Animate_Cartpole(t_all,x_traj,length_pole,animate_traj,X_REF);
    Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,X_REF_Original,U_REF_Original);
    
    
end


