function [] = Animate_Cartpole(t,x_traj,length_pole, animate_traj,X_REF)
if animate_traj
    if nargin == 4  
        %% Regulator problem
        figure
        for i = 1:length(t)/2
            clf;
            x = x_traj(1,i);
            th = x_traj(2,i);
            
            % Cart
            w = 1.5;
            h = 0.2;
            x_cart = x - w/2;
            y_cart = 0;
            
            pos = [x_cart y_cart w h];
            rectangle('Position',pos)
            
            % Pole
            pos_pole = [x+length_pole*cos(th-pi/2);
                length_pole*sin(th-pi/2)];    % theta axis points straight down in model so sub pi/2
            
            line([x pos_pole(1)],[h h+pos_pole(2)],'LineWidth',8);
            
            % Reset
            axis([x-1.5 x+1.5 0 1])
            drawnow;
            %             pause(0.001);
        end
    else    
        %% Trajectory Tracking Problem
        figure
        xlim([X_REF(1,end)-1.5 X_REF(1,end)+1.5])
        for i = 1:length(t)/2
            clf;
            x = x_traj(1,i);
            th = x_traj(2,i);
            x_ref = X_REF(1,i);
            th_ref = X_REF(2,i);
            
            
            % Cart
            w = 1.5;
            h = 0.2;
            x_cart = x - w/2;
            y_cart = 0;
            xref_cart = x_ref - w/2;
            yref_cart = 0;
            
            pos = [x_cart y_cart w h];
            pos_ref = [xref_cart yref_cart w h];
            r = rectangle('Position',pos,'EdgeColor','b','LineWidth',2);
            r_ref = rectangle('Position',pos_ref,'EdgeColor','g','LineWidth',2,'LineStyle','--');
            
            % Pole
            pos_pole = [x+length_pole*cos(th-pi/2);
                length_pole*sin(th-pi/2)];    % theta axis points straight down in model so sub pi/2
            
            posref_pole = [x_ref+length_pole*cos(th_ref-pi/2);
                length_pole*sin(th_ref-pi/2)];
            
            
            line([x pos_pole(1)],[h h+pos_pole(2)],'LineWidth',2);
            line([x_ref posref_pole(1)],[h h+posref_pole(2)],'LineWidth',2,'LineStyle','--','color','g')
            
            % Reset
%             axis([-1.5 x+1.5 0 1])
            x_offset = 6;
            xlim([X_REF(1,end)-x_offset X_REF(1,end)+x_offset])
            ylim([0 1.5]);
            drawnow;
            %         pause(0.001);
            
        end
    end
end