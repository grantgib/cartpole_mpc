function [x_traj,u_traj,x_traj_all,t_all,mpciter,args] = ...
    Simulate_NMPC(x_init,DT,N,n_x,n_c,f_nonlinear,solver,sim_time,...
                  type_reg,X_REF_Original,U_REF_Original)
% Choose regulator vs. trajectory tracking

import casadi.*
%% Initialize Variables
t_current = 0;          % current time step (sec)
t_all(1) = t_current;  
X_REF = X_REF_Original;
U_REF = U_REF_Original;

X0 = repmat(x_init,1,N+1)'; % initialization of the states decision variables.
%   This initialization assumes the state remains the same for each time
%   step
U0 = zeros(N,n_c);      % Initialize control decision variables to zero for
%  eachtime step

x_traj(:,1) = x_init;   % true trajectory states
x_traj_all = [];    % Contains predicted trajectories for each time step
u_traj=[];          % Control input trajectories

mpciter = 1;        % simulation iteration number (proportional to t_current)
main_loop = tic;    % Begin timer


%% Main Loop

while(mpciter < sim_time / DT)
    % the main simulaton loop... it works as long as the error is greater
    % than 10^-6 and the number of mpc steps is less than its maximum
    % value.
    %% Set Parameter vector and Decision Variables
    linearize = false;
    if type_reg
        args = Update_Parameters(x_init,N,n_x,n_c,X_REF,U_REF,type_reg,linearize); % X_REF is one state
    else
        args = Update_Parameters(x_init,N,n_x,n_c,X_REF,U_REF,type_reg,linearize);
    end
    
    args.x0  = [reshape(X0',n_x*(N+1),1);reshape(U0',n_c*N,1)];
    
    %% Solve MPC NLP solver (uses IPOPT)
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    
    %% Extract solutions
    % extract controls from the solution
    u = reshape(full(sol.x(n_x*(N+1)+1:end))',n_c,N)'; 
    
    % get solution TRAJECTORY
    x_traj_all(:,1:n_x,mpciter)= reshape(full(sol.x(1:n_x*(N+1)))',n_x,N+1)'; 
    
    % controller trajectory only uses first control input (could be
    % modified)
    u_traj= [u_traj ; u(1,:)];              

    %% Apply the control and guess next solution by shifting
    % vector containing all time steps
    t_all(mpciter) = t_current; 
    
    % Predict next step with Forward Euler Discretization
    [t_next, x_next, u_next_guess] = Update_State(DT, t_current, x_init, u, f_nonlinear);
    t_current = t_next;
    x_init = x_next;
    U0 = u_next_guess;
    
    % Update trajectory
    x_traj(:,mpciter+1) = x_init;
    
    % Warm Start: Store solution trajectory as next guess of state decision
    % variables. Shift trajectory to initialize the next step
    X0 = reshape(full(sol.x(1:n_x*(N+1)))',n_x,N+1)';
    X0 = [X0(2:end,:);X0(end,:)];   % initialize with next step and add on last state twice
    
    % Shift X_REF and U_REF ** Only for trajectory tracking**
    if ~type_reg
        X_REF = [X_REF(:,2:end),X_REF(:,end)];
        U_REF = [U_REF(:,2:end),U_REF(:,end)];
    end
    % Print every n iterations
    if mod(mpciter,50) == 0
        disp("MPC iteration = " + mpciter);
    end
    
    % Update iteration counter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);    
t_all(end+1) = t_all(end) + DT;     % update time since you simulated forward last control input
u_traj = u_traj';

%% Final Calculations
ss_error = norm((x_traj(:,end)-X_REF),2);
average_mpc_time = main_loop_time/(mpciter+1);
if ~type_reg
    traj_error = vecnorm((x_traj - X_REF_Original(:,1:size(x_traj,2)))');
    disp("Trajectory error (2-norm) = ")
    disp(traj_error);
end
disp("Steady-state error (2-norm) = " + ss_error);
disp("Average MPC Calculation Time = " + average_mpc_time);




end