function [] = Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,X_REF,U_REF)
if ~plot_traj
    return
end
figure
sz = 15;
width = 1;
u_min = args.lbx(end);
u_max = args.ubx(end);
if nargin == 8  % Regulator Problem
    x_ref = X_REF;
    subplot(2,3,1);
    yline(x_ref(1),'--g','LineWidth',width);
    hold on; plot(t_all,x_traj(1,:),'LineWidth',width);
%     hold on; yline(args.lbx(1),'--r','LineWidth',width);
%     hold on; yline(args.ubx(1),'--r','LineWidth',width);
    title('$\mathbf{x}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$x\ [m]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,2);
    yline(x_ref(2),'--g','LineWidth',width);
    hold on; plot(t_all,x_traj(2,:),'LineWidth',width);
    hold on; yline(args.lbx(2),'--r','LineWidth',width)
    hold on; yline(args.ubx(2),'--r','LineWidth',width)
    title('$\mathbf{\theta}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]');ylabel('$\theta\ [rad]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,4);
    yline(x_ref(3),'--g','LineWidth',width); 
    hold on; plot(t_all,x_traj(3,:),'LineWidth',width); 
    hold on; yline(args.lbx(3),'--r','LineWidth',width)
    hold on; yline(args.ubx(3),'--r','LineWidth',width)
    title('$\mathbf{\dot{x}}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$\dot{x}\ [m/s]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,5);
    yline(x_ref(4),'--g','LineWidth',width); 
    hold on; yline( args.lbx(4)'--r','LineWidth',width)
    hold on; yline(args.ubx(4),--'r','LineWidth',width)
    hold on; plot(t_all,x_traj(4,:),'LineWidth',width); 
    title('$\mathbf{\dot{\theta}}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$\dot{\theta}\ [rad/s]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz);
    
    subplot(2,3,3);
    yline(0,'--g','LineWidth',width); 
    hold on; stairs(t_all(1:end-1),u_traj,'LineWidth',width); 
    hold on; yline(u_min,'--r','LineWidth',width)
    hold on; yline(u_max,'--r','LineWidth',width)
    title('$\mathbf{u}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$u\ [Nm]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    set(gcf,'color','w');
    
else    % Trajectory Tracking problem
    sz = 15;
    width = 1;
    subplot(2,3,1);
    plot(t_all(1:mpciter),X_REF(1,1:mpciter),'--g','LineWidth',width); 
%     hold on; yline(args.lbx(1),'--r','LineWidth',width);
%     hold on; yline(args.ubx(1),'--r','LineWidth',width);
    hold on; plot(t_all,x_traj(1,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]);
    title('$\mathbf{x}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$x\ [m]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,2);
    plot(t_all(1:mpciter),X_REF(2,1:mpciter),'--g','LineWidth',width); 
    hold on; yline(args.lbx(2),'--r','LineWidth',width)
    hold on; yline(args.ubx(2),'--r','LineWidth',width)
    hold on; plot(t_all,x_traj(2,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); 
    title('$\mathbf{\theta}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]');ylabel('$\theta\ [rad]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,4);
    plot(t_all(1:mpciter),X_REF(3,1:mpciter),'--g','LineWidth',width); 
    hold on; yline(args.lbx(3),'--r','LineWidth',width)
    hold on; yline(args.ubx(3),'--r','LineWidth',width)
    hold on; plot(t_all,x_traj(3,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); 
    title('$\mathbf{\dot{x}}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$\dot{x}\ [m/s]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz)
    
    subplot(2,3,5);
    plot(t_all(1:mpciter),X_REF(4,1:mpciter),'LineStyle','--','color','g','LineWidth',width); 
    hold on; yline(args.lbx(4),'--r','LineWidth',width)
    hold on; yline(args.ubx(4),'--r','LineWidth',width)
    hold on; plot(t_all,x_traj(4,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); 
    title('$\mathbf{\dot{\theta}}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$\dot{\theta}\ [rad/s]$','interpreter','latex'); xlim([0 sim_time]);
    set(gca,'FontSize',sz);
    
    subplot(2,3,3);
    plot(t_all(1:mpciter-1),U_REF(1,1:mpciter-1),'--g','LineWidth',width)
    hold on; yline(u_min,'--r','LineWidth',width)
    hold on; yline(u_max,'--r','LineWidth',width)
    hold on; stairs(t_all(1:end-1),u_traj,'LineWidth',width,'color',[0, 0.4470, 0.7410]); 
    title('$\mathbf{u}$','interpreter','latex','FontSize',sz); grid on;
    xlabel('Time [sec]'); ylabel('$u\ [Nm]$','interpreter','latex');
    set(gca,'FontSize',sz)
    set(gcf,'color','w');
        
end

end