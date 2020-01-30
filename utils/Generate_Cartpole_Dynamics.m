function [f_nonlinear, n_x, n_c, l_pole] = Generate_Cartpole_Dynamics()

import casadi.*

% Symbolic variables
y = SX.sym('y'); 
theta = SX.sym('theta');
dy = SX.sym('dx'); dtheta = SX.sym('dtheta');
q = [y; theta]; dq = [dy; dtheta];
x = [q; dq]; n_x = length(x);

u = SX.sym('u');
u_ctrl = u; n_c = length(u_ctrl);

% Cartpole parameters
mc = 1;     % mass of cart (kg)
mp = 1;     % mass of pole (kg)
l_pole = 0.5;    % length of pole (m)
g = 9.81;   % gravity (m/s^2)

% Euler-Lagrange Dynamics: Pinned model
%       (D*ddq + C*dq + G = B*u)
D = [mc+mp mp*l_pole*cos(theta); mp*l_pole*cos(theta) mp*l_pole^2];
C = [0 -mp*l_pole*dtheta*sin(theta); 0 0];
G = [0; mp*g*l_pole*sin(theta)];
B = [1; 0];

% Time Invariant Nonlinear Dynamical System
xdot = [dq; D\(B*u-C*dq-G)]; % system

% Casadi Function
f_nonlinear = Function('f',{x,u_ctrl},{xdot});  % nonlinear mapping function f(x,u)


end