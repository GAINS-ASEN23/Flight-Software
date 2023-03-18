function [noisy_accel, ideal_accel] = sample_thrust(dt)
    % Returns accelerations in g's, not thrust in newtons

    mass_themis = 126;                      % Mass of the Themis Satellites [kg]

    %% Create Noise
    timevec = 1:dt:(60*60*2);
    noisevec = randn(1,length(timevec));
    func = @(x) noisevec(x);

    %% Create Thrust Curve
    start = 0:dt:20;
    val1 = zeros(size(start));

    leadup = 20:dt:30;
    val2 = -0.4*(20-leadup);

    peak = 30:dt:83;
    val3 = val2(end)*ones(size(peak));

    leadout = 83:dt:93;
    val4 = 0.4*(93-leadout);

    ending = 93:dt:113;
    val5 = zeros(size(ending));


    %% Add white noise
    sn_white1 = awgn(val1,5);
    sn_white2(1,1) = sn_white1(end);
    sn_white2(1,2:size(val2,2)) = awgn(val2(2:end),5);
    sn_white3(1,1) = sn_white2(end);
    sn_white3(1,2:size(val3,2)) = awgn(val3(2:end),5);
    sn_white4(1,1) = sn_white3(end);
    sn_white4(1,2:size(val4,2)) = awgn(val4(2:end),5);
    sn_white5(1,1) = sn_white4(end);
    sn_white5(1,2:size(val5,2)) = awgn(val5(2:end),5);


    %% Plot the Thrust
    figure;
    plot(start,val1,'--b','LineWidth',1);
    hold on; grid minor;

    plot(leadup,val2,'--b','LineWidth',1,'HandleVisibility','off');
    plot(peak,val3,'--b','LineWidth',1,'HandleVisibility','off');
    plot(leadout,val4,'--b','LineWidth',1,'HandleVisibility','off');
    plot(ending,val5,'--b','LineWidth',1,'HandleVisibility','off');

    plot(start,sn_white1,'-r','LineWidth',1.5);
    plot(leadup,sn_white2,'-r','LineWidth',1.5,'HandleVisibility','off');
    plot(peak,sn_white3,'-r','LineWidth',1.5,'HandleVisibility','off');
    plot(leadout,sn_white4,'-r','LineWidth',1.5,'HandleVisibility','off');
    plot(ending,sn_white5,'-r','LineWidth',1.5,'HandleVisibility','off');
    xlim([0 113]);
    xlabel('Time [sec]');
    ylabel('Thrust [N]'); 
    title('Thrust Curve');
    legend('Ideal Thrust Curve','Measured Thrust Curve');
    
    ideal_accel = [val1'; val2'; val3'; val4'; val5']./mass_themis;
    noisy_accel = [sn_white1'; sn_white2'; sn_white3'; sn_white4'; sn_white5']./mass_themis;
end