function [t_next, x_next, u_next_guess] = Update_State(DT, t_current, x_init,U_init,f)
st = x_init;
ctrl = U_init(1,:)';
f_value = f(st,ctrl);
x_next = full(st+ (DT*f_value));

t_next = t_current + DT;
u_next_guess = [U_init(2:size(U_init,1),:);U_init(size(U_init,1),:)];
end