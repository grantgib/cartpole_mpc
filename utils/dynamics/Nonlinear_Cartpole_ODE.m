function f = Nonlinear_Cartpole_ODE(q,dq,x,u,D,C,G,B,pinned)
import casadi.*
if pinned
    ddq = D\(-C*dq-G+B*u);
    xdot = [dq; ddq]; % system
    % Casadi Function
    f = Function('f_nonlinear',{x,u},{xdot});    % nonlinear mapping function f(x,u)
else % Floating Base
    % empty for now
end
end