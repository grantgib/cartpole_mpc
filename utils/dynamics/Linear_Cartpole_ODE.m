 function f_linear = Linear_Cartpole_ODE(q,dq,x,u,D,C,G,B,pinned)
import casadi.*

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

if pinned
        ddq = D\(-C*dq-G+B*u); % Create ddq symbolics for analytical linearization
        Alin = jacobian([dq; ddq],x);
        Blin = jacobian([dq; ddq],u);
        delta_xdot = Alin*delta_x + Blin*delta_u;
        f_linear = Function('f_linear',{x,u,delta_x,delta_u},{delta_xdot});    % linearized mapping (using Casadi jacobian)    
else % Floating Base Model
    % empty for now
end
















