function plot_1_3D(state1, labels)
    % Plots the 3D positions of two different state vectors composed as [x y z dx dy dz]
    % labels = string array   1: Title   2: Legend1

    figure;
    hold on
    sgtitle(labels(1), 'interpreter', 'latex', 'FontWeight', 'bold');

    p31 = plot3(state1(:,1), state1(:,2), state1(:,3), 'b', 'LineWidth', 1.5);

    xlabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    zlabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    axis equal;
        
end