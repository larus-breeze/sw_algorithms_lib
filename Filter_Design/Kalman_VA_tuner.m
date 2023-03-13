%clear all;
clear figures;
format long;

var_meas_v = 0.2^2; % velocity measurement variance 
var_meas_a = 0.05^2;  % acceleration measurement variance

header_length = 100;
numb = 2000;

initial_velocity_error = 1;

T = 0.01;               % sampling time

acceleration_process_noise = 3;

acc = acceleration_process_noise * randn( 1, numb);

vel = cumtrapz( acc) * T;
pos = cumtrapz( vel) * T;

A=[ 1 T ; 
    0 1 ]; 

C=[ 1 0; 
    0 1];

% obs_m=obsv(A, C);
% rank( obs_m);         % check observability

size = length(pos);

vpa = acceleration_process_noise^2; % acceleration process variance / (m/s²)²

Q = [T^5/20 * vpa  T^4/8;
     T^4/8  * vpa  T^3/3];

 % add measurement noise
acc_m = acc + sqrt( var_meas_a) * randn( 1, size);
vel_m = vel + sqrt( var_meas_v) * randn( 1, size);

R=[ var_meas_v 0;
    0 var_meas_a];    % measurement noise covariance matrix

x = [ initial_velocity_error 0]'; % system start state
P = A * Q * A' + Q;     % error covariance initialization 

vel_est = zeros( 1, size);
acc_est = zeros( 1, size);
gain1  = zeros( size);
gain2  = zeros( size);

for i = 1 : size
    P = A * P * A' + Q;
    K = P * C' / (C*P*C'+ R);
    x = A * x;
    x = x + K * ( [vel_m(i), acc_m(i)]' - C * x);
    P = ( eye(2) - K * C) * P;
    
    vel_est(i)   = [1 0] * x;
    acc_est(i)   = [0 1] * x;
    
    gain1(i)  = K(1,1);
    gain2(i)  = K(2,2);

end  

%subplot( 3, 1, 1);
figure(1);
plot(acc_est, 'r');
hold;
plot(acc,'g');
plot(acc_m, '+');
legend('Estimated Acc.','True Acc.','Acc.Measurement');
title('Kalmanfilter Movement Estimation');
ylabel('Accel. / m/s/s');
grid;
%subplot( 3, 1, 2);
figure(2);
plot(vel_est,'r');
hold;
plot(vel,'g');
plot(vel_m, '+');
legend('Estimated Velocity','True Velocity','Velocity Measurement');
ylabel('Velocity / m/s');
grid;
%subplot( 3, 1, 3);
figure(4);
plot(gain1,'b');
hold;
plot(gain2,'g');

K

velocity_error = std(vel-vel_est)
acc_error      = std(acc-acc_est)
acceleration_process_noise = std( acc)