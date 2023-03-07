function [accel_inertial, quat_next] = Accel_Inertial(accel_body, gyro, st, quat_prev)
% fuction takes in acceleration in the body and the qaterion inital given
% by the star tracker then updated due to the quat rate of change*dt (fixed
% at 100Hz)

% accel_body is the three acelerations in b1,b2,b3
% gryo is the angular rate in the body frame
% st is a quaternion from the star tracker
% quat_prev is the quaterion derrived from the previous quaterion rate
%   times dt @ 100 hz
%   [beta0 beta1 beta2 beta3] beta0 being the angle, beta1,2,3 being the
%   vector
% accel_inertial is the acceleration in the inertial frame
% quat_next is the current quaterion rate times dt to be sent into the next
%   iteration

dt = 1/100; %seconds

% forming the beta matrix from incoming quaternion in order to find
%   quaterion rate
beta_matrix = [quat_prev(1) -quat_prev(2) -quat_prev(3) -quat_prev(4); quat_prev(2) quat_prev(1) -quat_prev(4) quat_prev(3); quat_prev(3) quat_prev(4) quat_prev(1) -quat_prev(2); quat_prev(4) -quat_prev(3) quat_prev(2) quat_prev(1)];

% Finding quaternion rate using previous quaternion and gyro measurements
beta_dot = beta_matrix*[0;gyro(1);gyro(2);gyro(3)];

% multiplying by dt to get euler parameters
beta_gyro = beta_dot.*dt;


% if statement to deterime is star tracker or gyro quaternion will be used
if isnan(st(1)) || isnan(st(2)) || isnan(st(3)) % means star tracker data is not avalible

    % make quaterion from gyro stuff
    quat_next = beta_gyro;
    
else
    
    % make quaternion from star tracker 
    quat_next = st;

end

% convert quaternion to a DCM
NB_matrix = [quat_next(1)^2+quat_next(2)^2-quat_next(3)^2-quat_next(4)^2, 2*(quat_next(2)*quat_next(3)+quat_next(1)*quat_next(4)), 2*(quat_next(2)*quat_next(4)-quat_next(1)*quat_next(3)); ...
    2*(quat_next(2)*quat_next(3)-quat_next(1)*quat_next(4)), quat_next(1)^2-quat_next(2)^2+quat_next(3)^2-quat_next(4)^2, 2*(quat_next(3)*quat_next(4)+quat_next(1)*quat_next(2)); ...
    2*(quat_next(2)*quat_next(4)+quat_next(1)*quat_next(3)), 2*(quat_next(3)*quat_next(4)-quat_next(1)*quat_next(2)), quat_next(1)^2-quat_next(2)^2-quat_next(3)^2+quat_next(4)^2];


% convert acceleration from body frame to inertial with DCM
accel_inertial = NB_matrix*accel_body';


end