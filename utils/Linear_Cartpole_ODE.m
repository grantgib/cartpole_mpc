function f = Linear_Cartpole_ODE(x,u,dq,ddq,D,C,G,B,pinned,analytical)
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
    if analytical
        delta_xdot_analytical = 0;
        f_linear_analytical = Function('f_linear_analytical',{x,u},{delta_xdot_analytical});   % linearized mapping using analytical derivation68
        f = f_linear_analytical;
    else  % Casadi version
        Alin = jacobian([dq; ddq],x);
        Blin = jacobian([dq; ddq],u);
        delta_xdot = Alin*delta_x + Blin*delta_u;
        f_linear = Function('f_linear',{x,u,delta_x,delta_u},{delta_xdot});    % linearized mapping (using Casadi jacobian)
        f = f_linear;
    end
else % Floating Base Model
    % empty for now
end