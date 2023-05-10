function [state] = ground_cw(n, U_n, x_n_n, dt, tnm1, tn)
%{
    Last Edited: Bennett Grow 3/31/23

    Inputs:
        n = mean motion
        U_n - Input Vector, measurable deterministic input to the system
        x_n_n - Current Estimated State Vector


    Outputs:
        state - The next time step Estimated State Vector
%}



    % Define the State Transition Matrix
    F = [4-3*cos(n*dt) 0 0 sin(n*dt)/n -2*cos(n*dt)/n+2/n 0; 6*(sin(n*dt)-(n*dt)) 1 0 -2*(1-cos(n*dt))/n (4*sin(n*dt)-3*n*dt)/n 0; 0 0 cos(n*dt) 0 0 sin(n*dt)/n; 3*n*sin(n*dt) 0 0 cos(n*dt) 2*sin(n*dt) 0; -6*n*(1-cos(n*dt)) 0 0 -2*sin(n*dt) 4*cos(n*dt)-3 0; 0 0 -n*sin(n*dt) 0 0 cos(n*dt)];
    
    % Define Control Matrix
    G_func = @(dt,n,t1,t2)reshape([1.0./n.^2.*sin((dt.*n)./2.0).^2.*2.0,1.0./n.^2.*(sin(dt.*n).*2.0-dt.*n.*2.0),0.0,sin(dt.*n)./n,(sin((dt.*n)./2.0).^2.*-4.0)./n,0.0,-1.0./n.^2.*(sin(dt.*n).*2.0-dt.*n.*2.0),1.0./n.^2.*sin((dt.*n)./2.0).^2.*8.0+t1.*t2.*3.0-t1.^2.*(3.0./2.0)-t2.^2.*(3.0./2.0),0.0,(sin((dt.*n)./2.0).^2.*4.0)./n,dt.*-3.0+(sin(dt.*n).*4.0)./n,0.0,0.0,0.0,1.0./n.^2.*sin((dt.*n)./2.0).^2.*2.0,0.0,0.0,sin(dt.*n)./n],[6,3]);
    G = G_func(dt,n,tnm1,tn);

    % Extrapolate the state
    x_n_p_1_n = F*x_n_n + G*U_n;

    % Output calculated values
    state = x_n_p_1_n;

end