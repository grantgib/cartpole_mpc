function args = Update_Parameters(x_init,N,n_x,n_c,args,X_REF, U_REF)

if nargin == 6 % ==> type_reg = true
    %% Regulator Problem
    
    args.p   = [x_init; X_REF]; % set the values of the parameters vector
    % initial value of the decision variables
    
else
    %% Trajectory Tracking Problem
    args.p(1:n_x) = x_init; % initial condition of the robot posture
    for k = 1:N %new - set the reference to track           
        args.p((k-1)*(n_x+n_c)+(n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x)) = ...
            X_REF(:,k);
        
        args.p((k-1)*(n_x+n_c)+(n_x+n_x+1):(k-1)*(n_x+n_c)+(n_x+n_x+n_c)) =...
            U_REF(:,k);
    end
end