%% Test Casadi Linearization versus Analytical Derivation
% CONCLUSION
% Casadi jacobian is equivalent to more tedious analytical pinned
% linearization. Can also used for floating base model. 
clear; clc;

%% Path setup
if isunix
    %     addpath('../../casadi-linux-matlabR2014b-v3.5.1');
    import casadi.*
else
    addpath('../Toolboxes/casadi-windows-matlabR2014b-3.5.1');
    import casadi.*
end
addpath(genpath('utils/'));

%% Original Symbolics
% Symbolic variables
y = SX.sym('y');
theta = SX.sym('theta');
dy = SX.sym('dx');
dtheta = SX.sym('dtheta');
q = [y; theta];
dq = [dy; dtheta];
x = [q; dq];
n_x = length(x);

u_force = SX.sym('u_force');
u = u_force;
n_c = length(u);

% Cartpole parameters
mc = 0.1;         % mass of cart (kg)
mp = 5;         % mass of pole (kg)
l_pole = 1;   % length of pole (m)
g = 9.81;       % gravity (m/s^2)

%% Euler-Lagrange Dynamics: Pinned model
%       (D*ddq + C*dq + G = B*u)
D = [mc+mp mp*l_pole*cos(theta); mp*l_pole*cos(theta) mp*l_pole^2];
C = [0 -mp*l_pole*dtheta*sin(theta); 0 0];
G = [0; mp*g*l_pole*sin(theta)];
B = [1; 0];

f_D = Function('f_D',{x},{D});
f_C = Function('f_C',{x},{C});
f_G = Function('f_G',{x},{G});
f_B = Function('f_B',{x},{B});

%% Linearize Cartpole
n_q = length(q);
n_c = length(u);

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

%% Analytical linearization
ddq = SX.sym('ddq',n_q,1);  % Must create ddq symbolics for analytical derivation
A1 = D;
A2 = 2*C;
A3 = jacobian(D*ddq + C*dq + G,q);
Alina = [zeros(n_q,n_q), eye(n_q);
    -A1\A3, -A1\A2];
Blina = [zeros(n_q,n_c); A1\B];
delta_xdot_analytical = Alina*delta_x + Blina*delta_u;
f_Alina = Function('f_Alina',{x,ddq,u},{Alina});
f_Blina = Function('f_Blina',{x,ddq,u},{Blina});
f_linear_analytical = Function('f_linear_analytical',{x,ddq,u,delta_x,delta_u},{delta_xdot_analytical});   % linearized mapping using analytical derivation68


%% Casadi Linearization
ddq = D\(-C*dq-G+B*u);
Alin = jacobian([dq; ddq],x);
Blin = jacobian([dq; ddq],u);
delta_xdot = Alin*delta_x + Blin*delta_u;
f_Alin = Function('f_Alin',{x,u},{Alin});
f_Blin = Function('f_Blin',{x,u},{Blin});
f_linear = Function('f_linear',{x,u,delta_x,delta_u},{delta_xdot});   % linearized mapping (using Casadi jacobian)

%% Numerical Check
ntest = 10000;
Error = zeros(ntest,1);
time_analytical = zeros(ntest,1);
time_casadi = zeros(ntest,1);

disp('Running Tests...');
for i = 1:ntest
% test numbers
q_test = rand(2,1);
dq_test = rand(2,1);
x_test = [q_test; dq_test];
u_test = rand(1,1);
delta_x_test = rand(4,1);
delta_u_test = rand;
D_test = f_D(x_test);
C_test = f_C(x_test);
G_test = f_G(x_test);
B_test = f_B(x_test);
ddq_test = D_test\(-C_test*x_test(3:4)-G_test+B_test*u_test);

% compute and print
f_Alina(x_test,ddq_test,u_test);
f_Alin(x_test,u_test);

f_Blina(x_test,ddq_test,u_test);
f_Blin(x_test,u_test);

tic
fa = f_linear_analytical(x_test,ddq_test,u_test,delta_x_test,delta_u_test);
time_analytical(i) = toc;

tic
fc = f_linear(x_test,u_test,delta_x_test,delta_u_test);
time_casadi(i) = toc;

Error(i) = norm(full(fa - fc));

if mod(i,ntest/10) == 0
    disp(i);
end
end

%% End of Test Results
error_total = norm(Error);
casadi_time = mean(time_casadi);
analytical_time = mean(time_analytical);

disp('End of Test!');
disp('Statistics');
disp("Error Norm = " + error_total);
disp("Casadi Mean Computation Time = " + casadi_time);
disp("Analytical Mean Computation Time = " + analytical_time);

figure
plot(time_casadi); hold on; plot(time_analytical);
legend('casadi','analytical');
ylabel('Computation time (sec)');
figure
plot(Error);
ylabel('2-norm error');

save('LinearizationTestResults');





