function [Xdec, Udec, P, obj, g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f,type_reg,linearize)
import casadi.*
    U = SX.sym('U',n_c,N);      % controls in R^N-1. subset of the decision variables
    X = SX.sym('X',n_x,(N+1));  % A vector that represents the states over the optimization problem.
    dU = SX.sym('dU',n_c,N);        % linearization
    dX = SX.sym('dX',n_x,(N+1));    % linearization
if linearize
    %% Linear Formulation
    Xdec = dX;
    Udec = dU;
    if type_reg
        % Regulator
    else
        %% Trajectory Tracking
        P = SX.sym('P',n_x + N*(n_x+n_c));
        % P = [dx_init | x_0ref | u_0ref | x_1ref | u_1ref | ... | x_N-1ref | u_N-1ref]
        % parameters (which include the initial state and the reference along the
        % predicted trajectory (reference states and reference controls))
        
        % Objective Function
        obj = 0;    % initialize objective function (scalar output)
        Q = diag([10 1 0 0]);    % stage penalty
        R = 0;                      % control penalty
        for k = 1:N
            st = dX(:,k);    % current delta state 
            ctrl = dU(:,k);  % current delta control
            
            % Running stage cost
            obj = obj + st'*Q*st + ctrl*R*ctrl;
        end
        
        % Equality Constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; dX(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            st = dX(:,k);                        % current state
            ctrl = dU(:,k);                      % current control
            st_ref = P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x));
            ctrl_ref = P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c));
            st_next = dX(:,k+1);                 % next state (from decision variables)
            delta_xdot = f(st_ref,ctrl_ref,st,ctrl);                  % Nonlinear dynamics propogation
            st_next_euler = st+ (DT*delta_xdot);      % Forward Euler Discretization prediction
            g = [g; st_next-st_next_euler];     % Update constraints vector
        end
    end
else
    %% Nonlinear Formulation
    Xdec = X;
    Udec = U;
    if type_reg == true
        %% Regulator Problem
        P = SX.sym('P',n_x + n_x);  % parameters. P = [x_init; x_ref]
        
        % Build Objective Function
        obj = 0; % Objective function
        Q = diag([10 1 0.01 0.01]);
        R = 0.001; % weighing matrices (controls)
        for k = 1:N
            st = X(:,k);
            ctrl = U(:,k);
            obj = obj+(st-P(n_x+1:2*n_x))'*Q*(st-P(n_x+1:2*n_x)) + ctrl'*R*ctrl; % calculate obj
        end
        
        % Equality constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; X(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            st = X(:,k);                        % current state
            ctrl = U(:,k);                      % current control
            st_next = X(:,k+1);                 % next state (from decision variables)
            xdot = f(st,ctrl);                  % Nonlinear dynamics propogation
            st_next_euler = st+ (DT*xdot);      % Forward Euler Discretization prediction
            g = [g; st_next-st_next_euler];     % Update constraints vector
        end
    else
        %% trajectory tracking problem
        P = SX.sym('P',n_x + N*(n_x+n_c));
        % P = [xinit | x_0ref | u_0ref | x_1ref | u_1ref | ... | x_N-1ref | u_N-1ref]
        % parameters (which include the initial state and the reference along the
        % predicted trajectory (reference states and reference controls))
        
        % Objective Function
        obj = 0;    % initialize objective function (scalar output)
        Q = diag([10 1 .01 .01]);    % stage penalty
        R = 0.001;                      % control penalty
        for k = 1:N
            st = X(:,k);    % current state
            ctrl = U(:,k);  % current control
            
            % Running stage cost
            obj = obj + ...
                (st-P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x)))'*Q*(st-P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x))) + ...
                (ctrl-P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)))'*R*(ctrl-P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)));
        end
        
        % Equality Constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; X(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            st = X(:,k);                        % current state
            ctrl = U(:,k);                      % current control
            st_next = X(:,k+1);                 % next state (from decision variables)
            xdot = f(st,ctrl);                  % Nonlinear dynamics propogation
            st_next_euler = st+ (DT*xdot);      % Forward Euler Discretization prediction
            g = [g; st_next-st_next_euler];     % Update constraints vector
        end
    end %if type_reg
end % if linearize
end %function