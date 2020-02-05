function [solver_L,args_L,solver_NL,args_NL] = Formulate_NLP(DT,N,n_x,n_c,f_linear,f_nonlinear,type_reg,X_REF,U_REF)
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
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

%% Absolute Bound constants
y_lb = -1e6;
y_ub = -y_lb;
theta_lb = pi - pi/2;
theta_ub = pi + pi/2;
dy_lb = -20;
dy_ub = -dy_lb;
dtheta_lb = -20;
dtheta_ub = -dtheta_lb;
x_lb = [y_lb, theta_lb, dy_lb, dtheta_lb];
x_ub = [y_ub, theta_ub, dy_ub, dtheta_ub];

if type_reg
    % more constraints on the generated reference trajectory
    u_max = 50;
    u_min = -u_max;
else
    u_max = 60;
    u_min = -u_max;
end

%% Linearization Formulation
linearize = true;
[dX,dU,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f_linear,type_reg,linearize);

% Settings
% Decision variables to optimize
DEC_variables = [reshape(dX,n_x*(N+1),1);reshape(dU,n_c*N,1)];

% Formulate nlp
nlp_prob = struct('f', obj, 'x', DEC_variables, 'g', g, 'p', P);

% Setup solver
solver_L = nlpsol('solver','ipopt',nlp_prob,opts);

% Bounds for decision variables and constraints. args is updated inside of
% simulation later so it is an output of this function
args_L = struct;
args_L.lbg(1:n_x*(N+1)) = 0; % Equality constraints
args_L.ubg(1:n_x*(N+1)) = 0; % Equality constraints

% delta_state bounds
for k=1:N
    for i = 1:n_x
        args_L.ubx(n_x*(k-1)+i) = x_ub(i) - X_REF(i,k);
        args_L.lbx(n_x*(k-1)+i) = x_lb(i) - X_REF(i,k);
    end
    
    args_L.ubx(n_x*(N+1)+k) = u_max - U_REF(1,k);
    args_L.lbx(n_x*(N+1)+k) = u_min - U_REF(1,k);
    
end

%% Nonlinear Formulation
clear linearize dX dU P obj g DEC_variables nlp_prob
linearize = false;
% Compute symbolic variables of quadratic program
[X,U,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f_nonlinear,type_reg,linearize);

% Settings
% Decision variables to optimize
DEC_variables = [reshape(X,n_x*(N+1),1);reshape(U,n_c*N,1)];

% Formulate nlp
nlp_prob = struct('f', obj, 'x', DEC_variables, 'g', g, 'p', P);

% Setup solver
solver_NL = nlpsol('solver', 'ipopt', nlp_prob,opts);

% Bounds for decision variables and constraints. args is updated inside of
% simulation later so it is an output of this function
args_NL = struct;
args_NL.lbg(1:n_x*(N+1)) = 0; % Equality constraints
args_NL.ubg(1:n_x*(N+1)) = 0; % Equality constraints

args_NL.lbx(1:n_x:n_x*(N+1),1) = y_lb;              %state y lower bound
args_NL.ubx(1:n_x:n_x*(N+1),1) = y_ub;              %state y upper bound
args_NL.lbx(2:n_x:n_x*(N+1),1) = theta_lb;          %state theta lower bound
args_NL.ubx(2:n_x:n_x*(N+1),1) = theta_ub;          %state theta upper bound
args_NL.lbx(3:n_x:n_x*(N+1),1) = dy_lb;             %state dy lower bound
args_NL.ubx(3:n_x:n_x*(N+1),1) = dy_ub;             %state dy upper bound
args_NL.lbx(4:n_x:n_x*(N+1),1) = dtheta_lb;         %state dtheta lower bound
args_NL.ubx(4:n_x:n_x*(N+1),1) = dtheta_ub;         %state dtheta upper bound

args_NL.lbx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_min;    % u lower bound
args_NL.ubx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_max;    % u upper bound


end