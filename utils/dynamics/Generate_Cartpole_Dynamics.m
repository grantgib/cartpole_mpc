function [f_linear, f_nonlinear,n_x, n_c, l_pole] = Generate_Cartpole_Dynamics(pinned)
import casadi.*

% Symbolic state variables
y = SX.sym('y');
theta = SX.sym('theta');
dy = SX.sym('dx');
dtheta = SX.sym('dtheta');
q = [y; theta];
dq = [dy; dtheta];
x = [q; dq];
n_x = length(x);

% Symbolic control variables
u_force = SX.sym('u_force');
u = u_force;
n_c = length(u);

% Cartpole parameters
mc = 0.1;         % mass of cart (kg)
mp = 5;         % mass of pole (kg)
l_pole = 1;   % length of pole (m)
g = 9.81;       % gravity (m/s^2)

if pinned
    %% Euler-Lagrange Dynamics: Pinned model
    %       (D*ddq + C*dq + G = B*u)
    D = [mc+mp mp*l_pole*cos(theta); mp*l_pole*cos(theta) mp*l_pole^2];
    C = [0 -mp*l_pole*dtheta*sin(theta); 0 0];
    G = [0; mp*g*l_pole*sin(theta)];
    B = [1; 0];
   
    % Differential Equation for Dynamical System
    f_linear = Linear_Cartpole_ODE(q,dq,x,u,D,C,G,B,pinned);
    f_nonlinear = Nonlinear_Cartpole_ODE(q,dq,x,u,D,C,G,B,pinned);
    
else
    %% Euler-Lagrange Dynamics: Floating-base model
    %       (D*ddq + C*dq + G = B*u + W*lambda)
    
    
end





















