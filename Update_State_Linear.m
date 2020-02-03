function [t_next, x_next, u_next_guess] = ...
    Update_State_Linear(DT, t_current, dx_init, du, f_linear,X_REF,U_REF)

st_ref = X_REF(:,1);
ctrl_ref = U_REF(:,1)';
dst = dx_init;
dctrl = du(1,:)';
f_value = f_linear(st_ref,ctrl_ref,dst,dctrl);
dst_next = dst + (DT*f_value);
x_next = full(dst_next) + X_REF(:,2);

t_next = t_current + DT;
u_next_guess = [du(2:size(du,1),:);du(size(du,1),:)] + U_REF(:,2:N+1)';

end
