function plot_1(state1, t, labels)
    % Plots the positions and velocities of a given state vector composed as [x y z dx dy dz]
    % labels = string array   1: Title 

    figure;
    sgtitle(labels(1),'interpreter', 'latex', 'FontWeight', 'bold');

    subplot(2,2,1)
    hold on;
    plot(t, state1(:,1), 'b');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')

    subplot(2,2,3)
    hold on;
    plot(t, state1(:,2), 'b');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    
%     subplot(3,2,5)
%     hold on;
%     plot(t, state1(:,3), 'b');
%     xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
%     ylabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    
    %  lg1 = legend('KF','GS','location','northeast','orientation','horizontal');

    subplot(2,2,2)
    hold on;
    plot(t, state1(:,4), 'b');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("${\dot{x}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')

    subplot(2,2,4)
    hold on;
    plot(t, state1(:,5), 'b');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("${\dot{y}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')
    
%     subplot(3,2,6)
%     hold on;
%     plot(t, state1(:,6), 'b');
%     xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
%     ylabel("${\dot{z}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')

    %  lg2 = legend('KF','GS','location','northeast','orientation','horizontal');


end