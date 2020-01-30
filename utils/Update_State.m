function [t_next, x_next, u_next_guess] = Update_State(DT, t0, x0, u,f)
st = x0;
con = u(1,:)';
f_value = f(st,con);
st = st+ (DT*f_value);
x_next = full(st);

t_next = t0 + DT;
u_next_guess = [u(2:size(u,1),:);u(size(u,1),:)];
end