
clear; clc; close all;
import casadi.*
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

% Symbolic control variables
u_force = SX.sym('u_force');
u = u_force;
n_c = length(u);

% Cartpole parameters
mc = 0.1;         % mass of cart (kg)
mp = 10;         % mass of pole (kg)
l_pole = 1;   % length of pole (m)
g = 9.81;       % gravity (m/s^2)

pinned = 1;
if pinned
    % Euler-Lagrange Dynamics: Pinned model
    %       (D*ddq + C*dq + G = B*u)
    D = [mc+mp mp*l_pole*cos(theta); mp*l_pole*cos(theta) mp*l_pole^2];
    
    C = [0 -mp*l_pole*dtheta*sin(theta); 0 0]
    C_new = Coriolis(D,q,dq)
    G = [0; mp*g*l_pole*sin(theta)];
    B = [1; 0];
end



%% Results
ntest = 100
for i = 1:ntest
C_Func = Function('C_Func',{q,dq},{C});
C_new_Func = Function('C_new_Func',{q,dq},{C_new});


q_test = rand(n_q,1);
dq_test = rand(n_q,1);

C_test = C_Func(q_test,dq_test);
C_new_test = C_new_Func(q_test,dq_test);

Error(i) = norm(full(C_test - C_new_test));

if mod(ntest,10) == 0
    disp(ntest)
end
end


plot(Error)

