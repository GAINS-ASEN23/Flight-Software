function plot_2_3D(state1, state2, labels)
    % Plots the 3D positions of two different state vectors composed as [x y z dx dy dz]
    % labels = string array   1: Title   2: Legend1   3: Legend2

    figure;
    hold on
    sgtitle(labels(1), 'interpreter', 'latex', 'FontWeight', 'bold');

    rad_moon = 1737447.78;                  % Radius of the Moon [m]
    [sx, sy, sz] = sphere(1000);
    surf(sx.*rad_moon, sy.*rad_moon, sz.*rad_moon, 'EdgeColor',[192/256 192/256 192/256]);

    p31 = plot3(state1(:,1), state1(:,2), state1(:,3), 'b', 'LineWidth', 1.5);
    p32 = plot3(state2(:,1), state2(:,2), state2(:,3), 'r', 'LineWidth', 1.5);

    xlabel("$x\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    ylabel("$y\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    zlabel("$z\ [m]$", 'interpreter', 'latex', 'FontWeight', 'bold')
    axis equal;
        
    legend([p31,p32],labels(2),labels(3), Location='southoutside', interpreter='latex', FontWeight='bold')

end