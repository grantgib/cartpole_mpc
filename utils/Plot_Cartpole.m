function [] = Plot_Cartpole(t_all,x_traj,u_traj,sim_time,args,plot_traj,mpciter,X_REF,U_REF)
if plot_traj
    figure
    sz = 5;
    width = 1;
    u_min = args.lbx(end,:);
    u_max = args.ubx(end,:);
    if nargin == 8  % Regulator Problem
        x_ref = X_REF;
        subplot(2,3,1);
        line([0 sim_time],[x_ref(1) x_ref(1)],'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(1) args.lbx(1)],'LineStyle','--','color','r','LineWidth',width);
        hold on; line([0 sim_time],[args.ubx(1) args.ubx(1)],'LineStyle','--','color','r','LineWidth',width);
        hold on; plot(t_all,x_traj(1,:),'LineWidth',width); title('$\mathbf{x}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$x\ [m]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,2);
        line([0 sim_time],[x_ref(2) x_ref(2)],'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(2) args.lbx(2)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(2) args.ubx(2)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(2,:),'LineWidth',width); title('$\mathbf{\theta}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]');ylabel('$\theta\ [rad]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,4);
        line([0 sim_time],[x_ref(3) x_ref(3)],'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(3) args.lbx(3)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(3) args.ubx(3)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(3,:),'LineWidth',width); title('$\mathbf{\dot{x}}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$\dot{x}\ [m/s]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,5);
        line([0 sim_time],[x_ref(4) x_ref(4)],'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(4) args.lbx(4)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(4) args.ubx(4)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(4,:),'LineWidth',width); title('$\mathbf{\dot{\theta}}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$\dot{\theta}\ [rad/s]$','interpreter','latex');
        set(gca,'FontSize',sz);
        
        subplot(2,3,3);
        line([0 sim_time],[0 0],'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on;stairs(t_all(1:end-1),u_traj,'LineWidth',width); title('$\mathbf{u}$','interpreter','latex','FontSize',sz); grid on;
        hold on; line([0 sim_time],[u_min u_min],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[u_max u_max],'LineStyle','--','color','r','LineWidth',width)
        xlabel('Time [sec]'); ylabel('$u\ [Nm]$','interpreter','latex');
        set(gca,'FontSize',sz)
        set(gcf,'color','w');
        
        %     sgtitle('Cart-pole Regulator Example 1');
    else    % Trajectory Tracking problem
        sz = 15;
        width = 1;
        subplot(2,3,1);
        plot(t_all(1:mpciter),X_REF(1,1:mpciter),'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(1) args.lbx(1)],'LineStyle','--','color','r','LineWidth',width);
        hold on; line([0 sim_time],[args.ubx(1) args.ubx(1)],'LineStyle','--','color','r','LineWidth',width);
        hold on; plot(t_all,x_traj(1,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); title('$\mathbf{x}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$x\ [m]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,2);
        plot(t_all(1:mpciter),X_REF(2,1:mpciter),'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(2) args.lbx(2)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(2) args.ubx(2)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(2,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); title('$\mathbf{\theta}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]');ylabel('$\theta\ [rad]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,4);
        plot(t_all(1:mpciter),X_REF(3,1:mpciter),'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(3) args.lbx(3)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(3) args.ubx(3)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(3,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); title('$\mathbf{\dot{x}}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$\dot{x}\ [m/s]$','interpreter','latex');
        set(gca,'FontSize',sz)
        
        subplot(2,3,5);
        plot(t_all(1:mpciter),X_REF(4,1:mpciter),'LineStyle','--','color','g','LineWidth',width); xlim([0 sim_time]);
        hold on; line([0 sim_time],[args.lbx(4) args.lbx(4)],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[args.ubx(4) args.ubx(4)],'LineStyle','--','color','r','LineWidth',width)
        hold on; plot(t_all,x_traj(4,:),'LineWidth',width,'color',[0, 0.4470, 0.7410]); title('$\mathbf{\dot{\theta}}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$\dot{\theta}\ [rad/s]$','interpreter','latex');
        set(gca,'FontSize',sz);
        
        subplot(2,3,3);
        plot(t_all(1:mpciter-1),U_REF(1,1:mpciter-1),'LineStyle','--','color','g','LineWidth',width)
        hold on; line([0 sim_time],[u_min u_min],'LineStyle','--','color','r','LineWidth',width)
        hold on; line([0 sim_time],[u_max u_max],'LineStyle','--','color','r','LineWidth',width)
        hold on; stairs(t_all(1:end-1),u_traj,'LineWidth',width,'color',[0, 0.4470, 0.7410]); title('$\mathbf{u}$','interpreter','latex','FontSize',sz); grid on;
        xlabel('Time [sec]'); ylabel('$u\ [Nm]$','interpreter','latex');
        set(gca,'FontSize',sz)
        set(gcf,'color','w');
        
        %     sgtitle('Cart-pole Regulator Example 1');
        
    end
    
end % if plot_traj

end