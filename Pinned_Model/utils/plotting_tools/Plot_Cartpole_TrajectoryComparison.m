function [] = Plot_Cartpole_TrajectoryComparison(t_all_L,x_traj_L,u_traj_L,error_L,args_L,mpciter_L,...
    t_all_NL,x_traj_NL,u_traj_NL,error_NL,args_NL,mpciter_NL,...
    X_REF,U_REF,...
    sim_time,plot_compare)
if ~plot_compare
    return;
end
sz = 15;
width = 1;
u_min = args_NL.lbx(end,:);
u_max = args_NL.ubx(end,:);
mpciter = mpciter_NL;
% lincolor = [0, 0.4470, 0.7410];
% nlcolor = [0, 0.4470, 0.7410];
fignum = get(gcf,'Number');

%% Positions
figure(fignum+1)
subplot(2,2,1);
plot(t_all_L(1:mpciter),X_REF(1,1:mpciter),'LineWidth',width); 
hold on; plot(t_all_L,x_traj_L(1,:),'LineWidth',width);
hold on; plot(t_all_NL,x_traj_NL(1,:),'LineWidth',width);
% hold on; yline(args_NL.lbx(1),'--r','LineWidth',width);
% hold on; yline(args_NL.ubx(1),'--r','LineWidth',width);
xlabel('Time [sec]'); ylabel('$x\ [m]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{x}$','interpreter','latex','FontSize',sz); grid on;
legend('reference','LMPC','NMPC');
set(gca,'FontSize',sz)

subplot(2,2,2);
plot(t_all_L(1:mpciter),X_REF(2,1:mpciter),'LineWidth',width); 
hold on; plot(t_all_L,x_traj_L(2,:),'LineWidth',width); 
hold on; plot(t_all_NL,x_traj_NL(2,:),'LineWidth',width); 
hold on; yline(args_NL.lbx(2),'--r','LineWidth',width);
hold on; yline(args_NL.ubx(2),'--r','LineWidth',width); 
xlabel('Time [sec]');ylabel('$\theta\ [rad]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\theta}$','interpreter','latex','FontSize',sz); grid on;
legend('reference','LMPC','NMPC');
set(gca,'FontSize',sz)

% Errors
subplot(2,2,3);
plot(t_all_L,error_L(1,:),'LineWidth',width);
hold on; plot(t_all_NL,error_NL(1,:),'LineWidth',width);
xlabel('Time [sec]'); ylabel('$x_{error}\ [m]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{x_{error}}$','interpreter','latex','FontSize',sz); grid on;
legend('linear','nonlinear');
set(gca,'FontSize',sz)

subplot(2,2,4);
plot(t_all_L,error_L(2,:),'LineWidth',width);
hold on; plot(t_all_NL,error_NL(2,:),'LineWidth',width);
xlabel('Time [sec]');ylabel('$\theta_{error}\ [rad]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\theta_{error}}$','interpreter','latex','FontSize',sz); grid on;
legend('linear','nonlinear');
set(gca,'FontSize',sz)
sgtitle('Position Errors');

%% Velocities
figure(fignum+2)
subplot(2,2,1);
plot(t_all_L(1:mpciter),X_REF(3,1:mpciter),'LineWidth',width); 
hold on; plot(t_all_L,x_traj_L(3,:),'LineWidth',width); 
hold on; plot(t_all_NL,x_traj_NL(3,:),'LineWidth',width); 
hold on; yline(args_NL.lbx(3),'--r','LineWidth',width);
hold on; yline(args_NL.ubx(3),'--r','LineWidth',width);
xlabel('Time [sec]'); ylabel('$\dot{x}\ [m/s]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\dot{x}}$','interpreter','latex','FontSize',sz); grid on;
legend('reference','LMPC','NMPC');
set(gca,'FontSize',sz)

subplot(2,2,2);
plot(t_all_L(1:mpciter),X_REF(4,1:mpciter),'LineWidth',width); 
hold on; plot(t_all_L,x_traj_L(4,:),'LineWidth',width); 
hold on; plot(t_all_NL,x_traj_NL(4,:),'LineWidth',width); 
hold on; yline(args_NL.lbx(4),'--r','LineWidth',width);
hold on; yline(args_NL.ubx(4),'--r','LineWidth',width);
xlabel('Time [sec]'); ylabel('$\dot{\theta}\ [rad/s]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\dot{\theta}}$','interpreter','latex','FontSize',sz); grid on;
legend('reference','LMPC','NMPC');
set(gca,'FontSize',sz);

% Errors
subplot(2,2,3);
plot(t_all_L,error_L(3,:),'LineWidth',width);
hold on; plot(t_all_NL,error_NL(3,:),'LineWidth',width);
xlabel('Time [sec]'); ylabel('$\dot{x}_{error}\ [m/s]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\dot{x}_{error}}$','interpreter','latex','FontSize',sz); grid on;
legend('linear','nonlinear');
set(gca,'FontSize',sz)

subplot(2,2,4);
plot(t_all_L,error_L(4,:),'LineWidth',width);
hold on; plot(t_all_NL,error_NL(4,:),'LineWidth',width);
xlabel('Time [sec]'); ylabel('$\dot{\theta}_{error}\ [rad/s]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{\dot{\theta}_{error}}$','interpreter','latex','FontSize',sz); grid on;
legend('linear','nonlinear');
set(gca,'FontSize',sz)
sgtitle('Velocity Errors');

%% Controls
figure(fignum+3)
plot(t_all_L(1:mpciter-1),U_REF(1,1:mpciter-1),'LineWidth',width);
hold on; stairs(t_all_L(1:end-1),u_traj_L,'LineWidth',width); 
hold on; stairs(t_all_NL(1:end-1),u_traj_NL,'LineWidth',width); 
hold on; yline(u_min,'--r','LineWidth',width);
hold on; yline(u_max,'--r','LineWidth',width);
xlabel('Time [sec]'); ylabel('$u\ [Nm]$','interpreter','latex'); xlim([0 sim_time]);
title('$\mathbf{u_{error}}$','interpreter','latex','FontSize',sz); grid on;
legend('reference','LMPC','NMPC');
set(gca,'FontSize',sz)


% set(gcf,'color','w');



end