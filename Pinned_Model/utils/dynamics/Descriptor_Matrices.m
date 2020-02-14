function [E_nonlinear,H_nonlinear,E_linear,H_linear] = Descriptor_Matrices(q,dq,x,u,n_q,D,C,G,B)
import casadi.*
%% Nonlinear
E_NL = [eye(n_q), zeros(n_q,n_q);
    zeros(n_q,n_q), D];
H_NL = [dq; -C*dq-G+B*u];
E_nonlinear = Function('E',{x},{E_NL});
H_nonlinear = Function('H',{x,u},{H_NL});

%% Linear

E_linear = 0;
H_linear = 0;

end