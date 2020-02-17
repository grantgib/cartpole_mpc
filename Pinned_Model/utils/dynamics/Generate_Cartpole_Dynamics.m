function [f_nonlinear,f_linear,E_nonlinear,H_nonlinear,E_linear,H_linear,DDQ_REF,n_x, n_u, l_pole] = Generate_Cartpole_Dynamics()
import casadi.*

% Symbolic control variables
u_force = SX.sym('u_force');
u = u_force;
n_u = length(u);

% Cartpole parameters
mc = 0.1;         % mass of cart (kg)
mp = 10;         % mass of pole (kg)
l_pole = 1;   % length of pole (m)
g = 9.81;       % gravity (m/s^2)

%% Euler-Lagrange Dynamics: Pinned model
%       (D*ddq + C*dq + G = B*u)
% Symbolic state variables
y = SX.sym('y');
theta = SX.sym('theta');
dy = SX.sym('dx');
dtheta = SX.sym('dtheta');
q = [y; theta];
dq = [dy; dtheta];
x = [q; dq];
n_q = length(q);
n_x = length(x);
D = [mc+mp mp*l_pole*cos(theta); mp*l_pole*cos(theta) mp*l_pole^2];
C = Coriolis(D,q,dq); % Analytical_C = [0 -mp*l_pole*dtheta*sin(theta); 0 0]
G = [0; mp*g*l_pole*sin(theta)];
B = [1; 0];

% Differential Equation for Dynamical System
[f_nonlinear, f_linear] = Cartpole_ODE(q,dq,x,u,D,C,G,B);

% Descriptor System: Use E and H matrices to propogate without needing inverse of mass
% inertia matrix
% E*xdot = H --> E(xk)*x_k1 = E(xk)*xk + DT*H(xk)
[E_nonlinear,H_nonlinear,E_linear,H_linear,DDQ_REF] = Descriptor_Matrices(q,dq,x,u,n_q,D,C,G,B);

end
