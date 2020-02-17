function [solver_NL,solver_L] = Formulate_NLP(DT,N,n_x,n_c,f_nonlinear,f_linear,E_nonlinear,E_linear,H_nonlinear,H_linear,DDQ_REF,type_reg)
% Formulate NLP
%   * Symbolically create the objective function and equality constraints
%   (dynamics)
%   * Set the settings for the IPOPT solver
%   * Set numerical bounds on the decision variables and equality
%   constraints

% Initialize Symbolics
import casadi.*

% IPOPT settings
opts = struct;
% opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
% opts.ipopt.acceptable_tol =1e-8;
% opts.ipopt.acceptable_obj_change_tol = 1e-6;

%% Nonlinear Formulation
clear linearize dX dU P obj g DEC_variables nlp_prob
linearize = false;
% Compute symbolic variables of quadratic program
[X,U,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f_nonlinear,E_nonlinear,H_nonlinear,DDQ_REF,type_reg,linearize);

% Settings
% Decision variables to optimize
DEC_variables = [reshape(X,n_x*(N+1),1);reshape(U,n_c*N,1)];

% Formulate nlp
nlp_prob = struct('f', obj, 'x', DEC_variables, 'g', g, 'p', P);

% Setup solver
solver_NL = nlpsol('solver', 'ipopt', nlp_prob,opts);

%% Linearization Formulation
linearize = true;
[dX,dU,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f_linear,E_linear,H_linear,DDQ_REF,type_reg,linearize);

% Settings
% Decision variables to optimize
DEC_variables = [reshape(dX,n_x*(N+1),1);reshape(dU,n_c*N,1)];

% Formulate nlp
nlp_prob = struct('f', obj, 'x', DEC_variables, 'g', g, 'p', P);

% Setup solver
solver_L = nlpsol('solver','ipopt',nlp_prob,opts);


end