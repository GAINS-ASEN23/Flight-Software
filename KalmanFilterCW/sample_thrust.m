function [noisy_accel, ideal_accel] = sample_thrust(dt)
    
    g = 9.80665;             % [m/s^2] Acceleration at sea level
    mass_themis = 126;     % Mass of the Themis Satellites [kg]

    % From characterize_noise.m
    R = 0.073696013257492;
    POW = -41.157778297496290;

    %% Create Thrust Curve
    max_g = 0.138;

    start = linspace(0,20,20/dt);
    val1 = zeros(size(start));

    leadup = linspace(20,30,10/dt);
    val2 = -(20*max_g/10)+leadup*max_g/10;

    peak = linspace(30,83,53/dt);
    val3 = max_g*ones(size(peak));

    leadout = linspace(83,93,10/dt);
    val4 = (93*max_g/10)-leadout*max_g/10;

    ending = linspace(93,113,20/dt);
    val5 = zeros(size(ending));

    %% g to m/s^2

%     val1 = val1 .* g;
%     val2 = val2 .* g;
%     val3 = val3 .* g;
%     val4 = val4 .* g;
%     val5 = val5 .* g;



    %% Add white noise

    sn_white1 = awgn(val1,R,POW);
    sn_white2(1,1) = sn_white1(end);
    sn_white2(1,2:size(val2,2)) = awgn(val2(2:end),R,POW);
    sn_white3(1,1) = sn_white2(end);
    sn_white3(1,2:size(val3,2)) = awgn(val3(2:end),R,POW);
    sn_white4(1,1) = sn_white3(end);
    sn_white4(1,2:size(val4,2)) = awgn(val4(2:end),R,POW);
    sn_white5(1,1) = sn_white4(end);
    sn_white5(1,2:size(val5,2)) = awgn(val5(2:end),R,POW);

    %% Plot the Thrust

    time = [start'; leadup'; peak'; leadout'; ending'];
    ideal_accel = [val1'; val2'; val3'; val4'; val5'];
    noisy_accel = [sn_white1'; sn_white2'; sn_white3'; sn_white4'; sn_white5'];

    figure;
    hold on;
    xlim([0 113]);
    xlabel('Time [sec]');
    ylabel('Acceleration [m/s^2]'); 
    title('Thrust Curve');
    plot(time, noisy_accel);
    plot(time, ideal_accel, LineWidth=1.5);
    legend('Measured Thrust Curve','Ideal Thrust Curve', Location='south');
end