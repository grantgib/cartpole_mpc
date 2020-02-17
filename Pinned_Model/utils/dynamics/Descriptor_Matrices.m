function [E_nonlinear,H_nonlinear,E_linear,H_linear,DDQ_REF] = Descriptor_Matrices(q,dq,x,u,n_q,D,C,G,B)
import casadi.*
%% Nonlinear
E_NL = [eye(n_q), zeros(n_q,n_q);
    zeros(n_q,n_q), D];
H_NL = [dq; -C*dq-G+B*u];
E_nonlinear = Function('E',{x},{E_NL});
H_nonlinear = Function('H',{x,u},{H_NL});

%% Linear
n_q = length(q);
n_c = length(u);

% Create delta symbolic state and control variables
delta_y = SX.sym('delta_y');
delta_theta = SX.sym('delta_theta');
delta_dy = SX.sym('delta_dx');
delta_dtheta = SX.sym('delta_dtheta');
delta_q = [delta_y; delta_theta];
delta_dq = [delta_dy; delta_dtheta];
delta_x = [delta_q; delta_dq];
delta_u_force = SX.sym('delta_u_force');
delta_u = delta_u_force;

% Analytical linearization - result from Taylor Series Expansion
ddq = SX.sym('ddq',n_q,1);  % Must create ddq symbolics for analytical derivation
A1 = D;
A2 = 2*C;
A3 = jacobian(D*ddq + C*dq + G,q);

E_L = blkdiag(eye(n_q),D);
H_L = [delta_dq; -A3*delta_q - A2*delta_dq + B*delta_u];

E_linear = Function('E_L',{x,u,ddq,delta_x,delta_u},{E_L});
H_linear = Function('H_L',{x,u,ddq,delta_x,delta_u},{H_L});

ddq_ref = D\(-C*x((n_q)+1:end)-G+B*u);
DDQ_REF = Function('DDQ_REF',{x,u},{ddq_ref});
end