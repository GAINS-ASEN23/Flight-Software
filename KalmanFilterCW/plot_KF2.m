function plot_KF2(plot_pos_vel, plot3_pos, state, t, state_GS, t_GS, rad_moon)
    
    if plot_pos_vel == true
        
        figure;
        %title("Position (m)")
        subplot(3,2,1)
        plot(t(1:length(state(:,1))), state(:,1));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,1));
        ylabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')

        subplot(3,2,3)
        plot(t(1:length(state(:,1))), state(:,2));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,2));
        ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
        
        subplot(3,2,5)
        plot(t(1:length(state(:,1))), state(:,3));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,3));
        ylabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
        
        %  lg1 = legend('KF','GS','location','northeast','orientation','horizontal');
   
        %title("Velocity (m)")
        subplot(3,2,2)
        plot(t(1:length(state(:,1))), state(:,4));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,4));
        ylabel("${\dot{x}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')

        subplot(3,2,4)
        plot(t(1:length(state(:,1))), state(:,5));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,5));
        ylabel("${\dot{y}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
        
        subplot(3,2,6)
        plot(t(1:length(state(:,1))), state(:,6));
        hold on;
        %  plot(t_GS(1:length(state_GS(:,1))), state_GS(:,6));
        ylabel("${\dot{z}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')
        xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')

        %  lg2 = legend('KF','GS','location','northeast','orientation','horizontal');

    end
    
    if plot3_pos == true
        figure;
        %  [sx, sy, sz] = sphere(1000);
        %  surf(sx.*rad_moon, sy.*rad_moon, sz.*rad_moon, 'EdgeColor',[192/256 192/256 192/256]);
        %  hold on
    
        %title('3D Position');
        p31 = plot3(state(:,1), state(:,2), state(:,3), 'b', 'LineWidth', 3);
        xlabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        zlabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
        axis equal;
            
        %p32 = plot3(state_GS(:,1), state_GS(:,2), state_GS(:,3), 'r', 'LineWidth', 1.5);
        % legend([p31,p32],'KF','GS', Location='southoutside')

    end


end