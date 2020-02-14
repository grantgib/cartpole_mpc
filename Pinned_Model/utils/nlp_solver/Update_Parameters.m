function args = Update_Parameters(x_init,N,n_x,n_c,X_REF,U_REF,type_reg,linearize)
%% Set Bounds
% Absolute Bound constants
y_lb = -1e6;
y_ub = -y_lb;
theta_lb = pi - pi/2;
theta_ub = pi + pi/2;
dy_lb = -20;
dy_ub = -dy_lb;
dtheta_lb = -20;
dtheta_ub = -dtheta_lb;

% Final bounds
    x_lb = [y_lb, theta_lb, dy_lb, dtheta_lb];
    x_ub = [y_ub, theta_ub, dy_ub, dtheta_ub];

%% Update args.p
if type_reg % Regulator Problem
    %% Regulator Problem
    % Set control bounds
    u_max = 30;
    u_min = -u_max;
    % Redefine reference state
    x_ref = repmat(X_REF,1,N+1);
    u_ref = repmat(U_REF,1,N);
    % compute parameters vector
    args = struct;
    args.p = [x_init; X_REF]; % set the values of the parameters vector
    % initial value of the decision variables
    
else
    %% Trajectory Tracking Problem
    % Set control bounds
    u_max = 65;
    u_min = -u_max;
    
    % Redefine reference state
    x_ref = X_REF;
    u_ref = U_REF;
    
    % Compute parameters vector
    args = struct;
    args.p(1:n_x) = x_init; % initial condition of the robot posture
    for k = 1:N %new - set the reference to          track
        args.p((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x)) = ...
            X_REF(:,k);
        
        args.p((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)) =...
            U_REF(:,k);
    end
end
%% args.lbg/ubg and args.lbx/ubx
if linearize
    % Bounds for decision variables and constraints. args is updated inside of
    % simulation later so it is an output of this function
    args.lbg(1:n_x*(N+1)) = 0; % Equality constraints
    args.ubg(1:n_x*(N+1)) = 0; % Equality constraints
    
    % delta_state bounds
    for k=1:N
        for i = 1:n_x
            args.ubx(n_x*(k-1)+i) = x_ub(i) - x_ref(i,k); % Time-varying bounds on delta state
            args.lbx(n_x*(k-1)+i) = x_lb(i) - x_ref(i,k);
        end
        args.ubx(n_x*(N+1)+k) = u_max - u_ref(k);  % Time varying bounds on delta control
        args.lbx(n_x*(N+1)+k) = u_min - u_ref(k);
    end
else
    % Bounds for decision variables and constraints. args is updated inside of
    % simulation later so it is an output of this function]
    args.lbg(1:n_x*(N+1)) = 0; % Equality constraints
    args.ubg(1:n_x*(N+1)) = 0; % Equality constraints
    
    for i = 1:n_x
        args.lbx(i:n_x:n_x*(N+1),1) = x_lb(i);              %state y lower bound
        args.ubx(i:n_x:n_x*(N+1),1) = x_ub(i);
    end
    args.lbx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_min;    % u lower bound
    args.ubx(n_x*(N+1)+1:n_c:n_x*(N+1)+n_c*N,1) = u_max;    % u upper bound
end

end







