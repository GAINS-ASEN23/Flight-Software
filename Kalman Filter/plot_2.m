function plot_2(state1, state2, t, labels)
    % Plots the positions and velocities of two different state vectors composed as [x y z dx dy dz]
    % labels = string array   1: Title   2: Legend1   3: Legend2

    figure;
    sgtitle(labels(1), 'interpreter', 'latex', 'FontWeight', 'bold');

    subplot(2,2,1)
    hold on;
    plot(t, state1(:,1), 'b');
    plot(t, state2(:,1), 'r');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')

    subplot(2,2,3)
    hold on;
    plot(t, state1(:,2), 'b');
    plot(t, state2(:,2), 'r');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    
%     subplot(3,2,5)
%     hold on;
%     plot(t, state1(:,3), 'b');
%     plot(t, state2(:,3), 'r');
%     xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
%     ylabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
%     
    %  lg1 = legend('KF','GS','location','northeast','orientation','horizontal');

    subplot(2,2,2)
    hold on;
    plot(t, state1(:,4), 'b');
    plot(t, state2(:,4), 'r');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("${\dot{x}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')

    subplot(2,2,4)
    hold on;
    plot(t, state1(:,5), 'b');
    plot(t, state2(:,5), 'r');
    xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("${\dot{y}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')
    
%     subplot(3,2,6)
%     hold on;
%     plot(t, state1(:,6), 'b');
%     plot(t, state2(:,6), 'r');
%     xlabel("${Time\ [s]}$", 'interpreter', 'latex', 'FontWeight', 'bold')
%     ylabel("${\dot{z}}\ [m/s]$", 'interpreter', 'latex', 'FontWeight','bold')

    %  lg2 = legend('KF','GS','location','northeast','orientation','horizontal');





end