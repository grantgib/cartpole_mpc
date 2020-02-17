function [Xdec,Udec,P,obj,g] = Compute_Objective_Constraints(DT,N,n_x,n_c,f,E,H,DDQ_REF,type_reg,linearize)
import casadi.*
U = SX.sym('U',n_c,N);      % controls in R^N-1. subset of the decision variables
X = SX.sym('X',n_x,(N+1));  % A vector that represents the states over the optimization problem.
dU = SX.sym('dU',n_c,N);        % linearization
dX = SX.sym('dX',n_x,(N+1));    % linearization
%% Define Penalties
Qy_reg   = 10;
Qth_reg  = 5;
Qdy_reg  = 0.01;
Qdth_reg = 0.01;
R_reg = 0.01;

Qy_traj   = 1;
Qth_traj  = 1;
Qdy_traj  = 0.001;
Qdth_traj = 0.001;
R_traj = 0.0001;

%% Constraint Computation
if linearize
    %% Linear Formulation
    Xdec = dX;
    Udec = dU;
    if type_reg
        %% Linear Regulator
        P = SX.sym('P',n_x + n_x);  % parameters. P = [x_init; x_ref]
        % P = [dx_init | x_0ref | u_0ref | x_1ref | u_1ref | ... | x_N-1ref | u_N-1ref]
        % parameters (which include the initial state and the reference along the
        % predicted trajectory (reference states and reference controls))
        % Objective Function
        obj = 0;    % initialize objective function (scalar output)
        Q = diag([Qy_reg Qth_reg Qdy_reg Qdth_reg]);    % stage penalty
        
        R = R_reg;                      % control penalty
        
        for k = 1:N
            delta_x_k = dX(:,k);    % current delta state
            delta_u_k = dU(:,k);  % current delta control
            
            % Running stage cost
            obj = obj + delta_x_k'*Q*delta_x_k + delta_u_k*R*delta_u_k;
        end
        
        % Equality Constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; dX(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            delta_x_k = dX(:,k);                        % current state
            delta_u_k = dU(:,k);                      % current control
            x_ref = P(n_x+1:end);
            u_ref = 0;
            delta_x_k_1 = dX(:,k+1);                 % next state (from decision variables)
            
            % E*xdot = H --> E(delta_xk)*x_k1 = E(delta_xk)*xk + DT*H(delta_xk,delta_uk)
            dynamics_constraint = E(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k)*delta_x_k_1 - ...
                E(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k)*delta_x_k - ...
                DT*H(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k);
            
            g = [g; dynamics_constraint];
            
            %*ARCHIVED*: Stopped Forward Euler method due to
            %matrix inversion in f(...)
%             delta_xdot = f(x_ref,u_ref,delta_x_k,delta_u_k);                  % Nonlinear dynamics propogation
%             delta_x_k_1_euler = delta_x_k+ (DT*delta_xdot);      % Forward Euler Discretization prediction
%             g = [g; delta_x_k_1 - delta_x_k_1_euler];     % Update constraints vector
        end
        
    else
        %% Linear Trajectory Tracking
        P = SX.sym('P',n_x + N*(n_x+n_c));
        % P = [dx_init | x_0ref | u_0ref | x_1ref | u_1ref | ... | x_N-1ref | u_N-1ref]
        % parameters (which include the initial state and the reference along the
        % predicted trajectory (reference states and reference controls))
        
        % Objective Function
        obj = 0;    % initialize objective function (scalar output)
        
        Q = diag([Qy_traj Qth_traj Qdy_traj Qdth_traj]);    % stage penalty
        
        R = R_traj;                      % control penalty
        
        for k = 1:N
            delta_x_k = dX(:,k);    % current delta state
            delta_u_k = dU(:,k);  % current delta control
            
            % Running stage cost
            obj = obj + delta_x_k'*Q*delta_x_k + delta_u_k*R*delta_u_k;
        end
        
        % Equality Constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; dX(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            x_ref = P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x));
            u_ref = P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c));
            ddq_ref = DDQ_REF(x_ref,u_ref);
            delta_x_k = dX(:,k);                        % current state
            delta_u_k = dU(:,k);                      % current control
            delta_x_k_1 = dX(:,k+1);                 % next state (from decision variables)
            
            % E*xdot = H --> E(delta_xk)*x_k1 = E(delta_xk)*xk + DT*H(delta_xk,delta_uk)
            dynamics_constraint = E(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k)*delta_x_k_1 - ...
                E(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k)*delta_x_k - ...
                DT*H(x_ref,u_ref,ddq_ref,delta_x_k,delta_u_k);
            
            g = [g; dynamics_constraint];
            
            % *ARCHIVED*: Stopped Forward Euler method due to
            % matrix inversion in f(...)
%             delta_xdot = f(x_ref,u_ref,delta_x_k,delta_u_k);                  % Nonlinear dynamics propogation
%             delta_x_k_1_euler = delta_x_k+ (DT*delta_xdot);      % Forward Euler Discretization prediction
%             g = [g; delta_x_k_1 - delta_x_k_1_euler];     % Update constraints vector
            
            
        end
    end
else
    %% Nonlinear Formulation
    Xdec = X;
    Udec = U;
    if type_reg
        %% Nonlinear Regulator Problem
        P = SX.sym('P',n_x + n_x);  % parameters. P = [x_init; x_ref]
        
        % Build Objective Function
        obj = 0; % Objective function
        Q = diag([Qy_reg Qth_reg Qdy_reg Qdth_reg]);    % stage penalty
        
        R = R_reg; % weighing matrices (controls)
        for k = 1:N
            x_k = X(:,k);
            u_k = U(:,k);
            obj = obj+...
                (x_k-P(n_x+1:2*n_x))'*Q*(x_k-P(n_x+1:2*n_x)) + ...
                u_k'*R*u_k; % calculate obj
        end
        
        % Equality constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; X(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            x_k = X(:,k);                        % current state
            u_k = U(:,k);                      % current control
            x_k_1 = X(:,k+1);                 % next state (from decision variables)
            g = [g; E(x_k)*x_k_1 - E(x_k)*x_k - DT*H(x_k,u_k)]; % E*xdot = H --> E(xk)*x_k1 = E(xk)*xk + DT*H(xk)
            % *ARCHIVED*: Stopped Forward Euler method due to
            % matrix inversion
            %             xdot = f(x_k,u_k);                  % Nonlinear dynamics propogation
            %             x_k_1_euler = x_k + (DT*xdot);      % Forward Euler Discretization prediction
            %             g = [g; x_k_1 - x_k_1_euler];     % Update constraints vector
            
        end
    else
        %% Nonlinear Trajectory Tracking
        P = SX.sym('P',n_x + N*(n_x+n_c));
        % P = [xinit | x_0ref | u_0ref | x_1ref | u_1ref | ... | x_N-1ref | u_N-1ref]
        % parameters (which include the initial state and the reference along the
        % predicted trajectory (reference states and reference controls))
        
        % Objective Function
        obj = 0;    % initialize objective function (scalar output)
        
        Q = diag([Qy_traj Qth_traj Qdy_traj Qdth_traj]);    % stage penalty
        
        R = R_traj;                      % control penalty
        
        for k = 1:N
            x_k = X(:,k);    % current state
            u_k = U(:,k);  % current control
            
            % Running stage cost
            obj = obj + ...
                (x_k-P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x)))'*Q*(x_k-P((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x))) + ...
                (u_k-P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)))'*R*(u_k-P((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)));
        end
        
        % Equality Constraints (Dynamics)
        g = [];                 % initialize equality constraints vector
        g = [g; X(:,1)-P(1:n_x)];   % initial condition constraints
        for k = 1:N
            x_k = X(:,k);                        % current state
            u_k = U(:,k);                      % current control
            x_k_1 = X(:,k+1);                 % next state (from decision variables)
            g = [g; E(x_k)*x_k_1 - E(x_k)*x_k - DT*H(x_k,u_k)]; % E*xdot = H --> E(xk)*x_k1 = E(xk)*xk + DT*H(xk)
            % *ARCHIVED*: Stopped Forward Euler method due to
            % matrix inversion
            %             xdot = f(x_k,u_k);                  % Nonlinear dynamics propogation
            %             x_k_1_euler = x_k + (DT*xdot);      % Forward Euler Discretization prediction
            %             g = [g; x_k_1 - x_k_1_euler];     % Update constraints vector
            
        end
    end %if type_reg
end % if linearize
end %function