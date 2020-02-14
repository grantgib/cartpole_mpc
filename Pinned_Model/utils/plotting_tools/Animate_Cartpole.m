function [] = Animate_Cartpole(t_all,x_traj,l_pole,animate_traj,type_reg,sim_time,DT,X_REF)
if ~animate_traj
    return
end

n = sim_time / DT;
if type_reg
    %% Regulator problem
    figure
    for i = 1:n

        clf;
        y = x_traj(1,i);
        th = x_traj(2,i);
        
        % Cart
        w = 1.0;
        h = 0.4;
        y_cart = y - w/2;
        z_cart = 0;
        r_ball = 0.2;
        
        pos = [y_cart z_cart w h];
        rect = rectangle('Position',pos);
        rect.FaceColor = [0 0.4470 0.7410];
        % Pole
        ps = [y; h];
        pe = [y + l_pole*cos(th-pi/2);
              h + l_pole*sin(th-pi/2)];    % theta axis points straight down in model so sub pi/2
        ball = [y + (l_pole + r_ball)*cos(th-pi/2);
                h + (l_pole + r_ball)*sin(th-pi/2)];
        circle(ball(1),ball(2),r_ball,[0 0.4470 0.7410]);
        hold on; line([ps(1) pe(1)], [ps(2) pe(2)],'LineWidth',2); % Draw pendulum
        
        
        
        % Draw pendulum point mass
        hold on; yline(0,'k','LineWidth',2);
        % Reset
        axis equal;
        xlim([X_REF(1)-3 X_REF(1)+3]);
        ylim([-1 2]);
        
        xlabel('y'); ylabel('z');
        drawnow;
        pause(0.001);
        
    end
else
    %% Trajectory Tracking Problem
    figure
    for i = 1:n
        clf;
        
        % Grab data
        y = x_traj(1,i);
        th = x_traj(2,i);
        y_ref = X_REF(1,i);
        th_ref = X_REF(2,i);
        
        % Cart parameters
        w = 1.0;
        h = 0.4;
        h_ref = h;
        y_cart = y - w/2;
        z_cart = 0;
        y_cart_ref = y_ref - w/2;
        z_cart_ref = 0;
        r_ball = 0.2;
        
        % Draw Cart
        pos = [y_cart z_cart w h];
        pos_ref = [y_cart_ref z_cart_ref w h];
        rectangle('Position',pos,'FaceColor',[0 0.4470 0.7410]);
        hold on; rectangle('Position',pos_ref,'FaceColor',[0.8500 0.3250 0.0980]);
        
        % Draw Pole
        pos_pole = [y+l_pole*cos(th-pi/2);
                    l_pole*sin(th-pi/2)];    % theta axis points straight down in model so sub pi/2
        ball = [y + (l_pole + r_ball)*cos(th-pi/2);
                h + (l_pole + r_ball)*sin(th-pi/2)];
        pos_pole_ref = [y_ref + l_pole*cos(th_ref-pi/2);
                        l_pole*sin(th_ref-pi/2)];
        ball_ref = [y_ref + (l_pole + r_ball)*cos(th_ref-pi/2);
                    h_ref + (l_pole + r_ball)*sin(th_ref-pi/2)];       
        hold on; line([y pos_pole(1)],[h h+pos_pole(2)],'LineWidth',2,'color',[0 0.4470 0.7410]);
        circle(ball(1),ball(2),r_ball,[0 0.4470 0.7410]);
        hold on; line([y_ref pos_pole_ref(1)],[h h+pos_pole_ref(2)],'LineWidth',2,'color',[0.8500 0.3250 0.0980]);
        circle(ball_ref(1),ball_ref(2),r_ball,[0.8500 0.3250 0.0980]);
        
        % Draw pendulum point mass
        hold on; yline(0,'k','LineWidth',2);
        % Reset
        axis equal;
        xlim([X_REF(1,end)-3 X_REF(1,end)+3]);
        ylim([-1 2]);
        
        xlabel('y'); ylabel('z');
        drawnow;
        pause(0.001);
        
    end
end



function h = circle(x,y,r,color)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,'color',color);