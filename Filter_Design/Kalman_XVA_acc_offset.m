%clear all;
clear figures;
format long;

var_meas_x = 0.1 ^2;  % position measurement variance 
var_meas_v = 0.15 ^2; % velocity measurement variance 
var_meas_a = 0.1 ^2;  % acceleration measurement variance

header_length = 100;
numb = 2000;

initial_position_error = 1;
initial_velocity_error = 1;

T = 0.01;               % sampling time

acceleration_process_noise = 0.1;
acceleration = 1; 

if 1 % test signal
    acc = [ zeros(1,header_length) acceleration * ones(1,100) zeros(1,500) -acceleration * ones(1,100) zeros(1,500)];
    acc = [ acc -acc];
else % just noise
    acc = acceleration_process_noise * randn( 1, numb);
end

vel = cumtrapz( acc) * T;
pos = cumtrapz( vel) * T;

A=[ 1 T T*T/2 0; 
    0 1 T 0; 
    0 0 1 0;
    0 0 0 1];             % system dynamics including acceleration offset

C=[ 1 0 0 0; 
    0 1 0 0;
    0 0 1 1];             % measurement

% obs_m=obsv(A, C);
% rank( obs_m);         % check observability

size = length(pos);

acc_offset = [ linspace( 0, 1, size/2) ones( 1, size/2)];  

vpa = 1.0^2; % acceleration process variance / (m/s²)²

%V = T * var_proc_a;
vaoff = 0.0001;

Q = [T^5/20 * vpa  T^4/8 * vpa  T^3/6 * vpa  0;
     T^4/8  * vpa  T^3/3 * vpa  T^2/2 * vpa  0;
     T^3/6  * vpa  T^2/2 * vpa  T     * vpa  0;
     0     0     0      vaoff];    % process noise covariance matrix

 % add measurement noise
acc_m = acc_offset + acc + sqrt( var_meas_a) * randn( 1, size);
vel_m = vel + sqrt( var_meas_v) * randn( 1, size);
pos_m = pos + sqrt( var_meas_x) * randn( 1, size);

R=[var_meas_x 0 0;
    0 var_meas_v 0;
    0 0 var_meas_a];    % measurement noise covariance matrix

x = [ initial_position_error  initial_velocity_error 0 -1]'; % system start state
P = A * Q * A' + Q;     % error covariance initialization 

pos_est  = zeros( 1, size);
vel_est = zeros( 1, size);
acc_est   = zeros( 1, size);
acc_off_est = zeros( 1, size);
gain1  = zeros( size);
gain2  = zeros( size);
gain3  = zeros( size);

for i = 1 : size
    P = A * P * A' + Q;
    K = P * C' / (C*P*C'+ R);
    x = A * x;
    x = x + K * ( [pos_m(i) vel_m(i), acc_m(i)]' - C * x);
    P = ( eye(4) - K * C) * P;
    
    pos_est(i)   = [1 0 0 0] * x;
    vel_est(i)   = [0 1 0 0] * x;
    acc_est(i)   = [0 0 1 0] * x;
    acc_est_x(i) = [0 0 0 1] * x;
    
    gain1(i)  = K(1,1);
    gain2(i)  = K(2,2);
    gain3(i)  = K(3,3);

end  

subplot( 3, 1, 1);
%figure(1);
plot(acc_est, 'r');
hold;
plot(acc,'g');
plot(acc_m, '.');
plot(acc_est_x);
plot(acc_offset);
legend('Estimated Acc.','True Acc.','Acc.Measurement','Acc. Offset est.','Acc. Offset');
title('Kalmanfilter Movement Estimation');
ylabel('Accel. / m/s/s');
grid;
subplot( 3, 1, 2);
%figure(2);
plot(vel_est,'r');
hold;
plot(vel,'g');
plot(vel_m, '.');
legend('Estimated Velocity','True Velocity','Velocity Measurement');
ylabel('Velocity / m/s');
grid;
subplot( 3, 1, 3)
%figure(3);
plot(pos_est, 'r');
hold;
plot(pos,'g');
plot(pos_m, '.');
ylabel('Position / m');
legend('Estimated Position','True Position','Position Measurement');
xlabel('Time / 10ms');
grid;
%subplot( 4, 1, 4);
figure(4);
plot(gain1,'b');
hold;
plot(gain2,'g');
plot(gain3,'r');

K

position_error = std(pos-pos_est)
velocity_error = std(vel-vel_est)
acc_error      = std(acc-acc_est)
acceleration_process_noise = std( acc)