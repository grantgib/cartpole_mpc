function [f_nonlinear,f_linear] = Cartpole_ODE(q,dq,x,u,D,C,G,B)
import casadi.*
% Pinned Model Assumption: D*ddq + C*dq + G = B*u

%% Nonlinear ODE
ddq = D\(-C*dq-G+B*u);
xdot = [dq; ddq]; % system

% ode functino rhs
f_nonlinear = Function('f_nonlinear',{x,u},{xdot});    % nonlinear mapping function f(x,u

%% Linear ODE
% Extra Symbolics: create delta symbolic state and control variables
delta_y = SX.sym('delta_y');
delta_theta = SX.sym('delta_theta');
delta_dy = SX.sym('delta_dx');
delta_dtheta = SX.sym('delta_dtheta');
delta_q = [delta_y; delta_theta];
delta_dq = [delta_dy; delta_dtheta];
delta_x = [delta_q; delta_dq];
delta_u_force = SX.sym('delta_u_force');
delta_u = delta_u_force;

% function creation (rhs)
ddq = D\(-C*dq-G+B*u); % Create ddq symbolics for analytical linearization
Alin = jacobian([dq; ddq],x);
Blin = jacobian([dq; ddq],u);
delta_xdot = Alin*delta_x + Blin*delta_u;
f_linear = Function('f_linear',{x,u,delta_x,delta_u},{delta_xdot});    % linearized mapping (using Casadi jacobian


end