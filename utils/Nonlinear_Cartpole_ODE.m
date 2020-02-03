function f = Nonlinear_Cartpole_ODE(x,u,dq,ddq,pinned)
import casadi.*
if pinned
    xdot = [dq; ddq]; % system
    % Casadi Function
    f = Function('f_nonlinear',{x,u},{xdot});    % nonlinear mapping function f(x,u)
else % Floating Base
    % empty for now
end
end