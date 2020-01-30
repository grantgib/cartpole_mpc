function [solver, args] = Formulate_NLP(DT, N,n_x,n_c,f_nonlinear,type_reg)
% Formulate NLP
%   * Symbolically create the objective function and equality constraints
%   (dynamics)
%   * Set the settings for the IPOPT solver
%   * Set numerical bounds on the decision variables and equality
%   constraints

%% Initialize Symbolics
import casadi.*             

[X,U,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f_nonlinear,type_reg);

%% Settings
% Decision variables to optimize
DEC_variables = [reshape(X,n_x*(N+1),1);reshape(U,n_c*N,1)];

% Formulate nlp
nlp_prob = struct('f', obj, 'x', DEC_variables, 'g', g, 'p', P);

% IPOPT settings
opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

% Setup solver
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% Bounds for decision variables and constraints. args is updated inside of
% simulation later so it is an output of this function
args = struct;
args.lbg(1:n_x*(N+1)) = 0; % Equality constraints
args.ubg(1:n_x*(N+1)) = 0; % Equality constraints

args.lbx(1:n_x:n_x*(N+1),1) = -inf;             %state x lower bound
args.ubx(1:n_x:n_x*(N+1),1) = inf;              %state x upper bound
args.lbx(2:n_x:n_x*(N+1),1) = 3*pi/4;             %state theta lower bound
args.ubx(2:n_x:n_x*(N+1),1) = 5*pi/4;           %state theta upper bound
args.lbx(3:n_x:n_x*(N+1),1) = -2;             %state dx lower bound
args.ubx(3:n_x:n_x*(N+1),1) = 2;              %state dx upper bound
args.lbx(4:n_x:n_x*(N+1),1) = -0.5;             %state dtheta lower bound
args.ubx(4:n_x:n_x*(N+1),1) = 0.5;              %state dtheta upper bound

u_max = 10; u_min = -u_max;
args.lbx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_min;    % u lower bound
args.ubx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_max;    % u upper bound














end