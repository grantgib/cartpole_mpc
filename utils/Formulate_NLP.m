function [solver, args] = Formulate_NLP(DT, N,n_x,n_c,f,type_reg,linearize)
% Formulate NLP
%   * Symbolically create the objective function and equality constraints
%   (dynamics)
%   * Set the settings for the IPOPT solver
%   * Set numerical bounds on the decision variables and equality
%   constraints

% Initialize Symbolics
import casadi.*
if linearize
    %% Linearization Formulation
    [dX,dU,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f,type_reg,linearize);
    
    % Settings
    % Decision variables to optimize
    DEC_variables = [reshape(dX,n_x*(N+1),1);reshape(dU,n_c*N,1)];
    
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
    solver = nlpsol('solver','ipopt',nlp_prob,opts);
    
    % Bounds for decision variables and constraints. args is updated inside of
    % simulation later so it is an output of this function
    args = struct;
    args.lbg(1:n_x*(N+1)) = 0; % Equality constraints
    args.ubg(1:n_x*(N+1)) = 0; % Equality constraints
    
    % delta_state bounds
    args.lbx(1:n_x:n_x*(N+1),1) = -0.1;             %state y lower bound
    args.ubx(1:n_x:n_x*(N+1),1) = 0.1;              %state y upper bound
    args.lbx(2:n_x:n_x*(N+1),1) = -0.1;             %state theta lower bound
    args.ubx(2:n_x:n_x*(N+1),1) = 0.1;           %state theta upper bound
    args.lbx(3:n_x:n_x*(N+1),1) = -4;%-3;             %state dy lower bound
    args.ubx(3:n_x:n_x*(N+1),1) = 4;%3;              %state dy upper bound
    args.lbx(4:n_x:n_x*(N+1),1) = -4; %-3;             %state dtheta lower bound
    args.ubx(4:n_x:n_x*(N+1),1) = 4; %3;              %state dtheta upper bound
    
    if type_reg
        % regulator
    else
        du_max = inf;
        du_min = -du_max;
    end
    args.lbx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = du_min;    % u lower bound
    args.ubx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = du_max;    % u upper bound
    
else
    %% Nonlinear Formulation
    % Compute symbolic variables of quadratic program
    [X,U,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f,type_reg);
    
    % Settings
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
    
    args.lbx(1:n_x:n_x*(N+1),1) = -inf;             %state y lower bound
    args.ubx(1:n_x:n_x*(N+1),1) = inf;              %state y upper bound
    args.lbx(2:n_x:n_x*(N+1),1) = pi-pi/2;             %state theta lower bound
    args.ubx(2:n_x:n_x*(N+1),1) = pi+pi/2;           %state theta upper bound
    args.lbx(3:n_x:n_x*(N+1),1) = -20;%-3;             %state dy lower bound
    args.ubx(3:n_x:n_x*(N+1),1) = 20;%3;              %state dy upper bound
    args.lbx(4:n_x:n_x*(N+1),1) = -20; %-3;             %state dtheta lower bound
    args.ubx(4:n_x:n_x*(N+1),1) = 20; %3;              %state dtheta upper bound
    
    if type_reg
        % more constraints on the generated reference trajectory
        u_max = 50;
        u_min = -u_max;
    else
        u_max = 60;
        u_min = -u_max;
    end
    args.lbx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_min;    % u lower bound
    args.ubx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_max;    % u upper bound
    
end







end