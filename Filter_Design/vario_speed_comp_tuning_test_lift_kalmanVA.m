samplerate=100; % Hz

tau = 0.75;
[B,A]=butter( 2, 1 / samplerate / tau, 'low');

acceleration = 10; 
g = 9.81;
acc_offset = - g;

header_length = 100;
acc = [ zeros(1,header_length) acceleration * ones(1,100) zeros(1,300) -acceleration * ones(1,100) zeros(1,header_length)];
acc = filter( B,A,acc);
%acc = [ acc -acc];
vel = cumtrapz( acc) / samplerate;
pos = cumtrapz( vel) / samplerate;

initial_speed = 50; % m/s
energy_altitude = initial_speed^2 / 2 / g;
horiz_speed = initial_speed; % sqrt( (energy_altitude - pos) * 2 * g - vel.^2)
horiz_acc = [ 0 diff( horiz_speed) * samplerate];

[kvario, kacc] = kalman_vario_XVA( pos, vel, acc + acc_offset);
[est_vel, est_a] = Kalman_VA( horiz_speed, horiz_acc);

speed_compensation = (est_vel .* est_a + kvario .* kacc) / g;
%speed_compensation = ( horiz_speed .* horiz_acc + kvario .* kacc) / g;
speed_comp_vario = filter(B,A, kvario + speed_compensation);

absolue_speed=sqrt(horiz_speed .^ 2 + vel .^2 );

samples=length( pos);
time=linspace(0,samples/samplerate,samples);

plot(time, pos,'LineWidth',2.0)
hold
grid
plot(time,kvario,'LineWidth',2.0);
plot(time,speed_comp_vario,'LineWidth',2.0);
plot(time,speed_compensation,'LineWidth',2.0);
plot(time,absolue_speed,'LineWidth',2.0);

axis([0 samples/samplerate -15 70]);
legend('Altitude','Kalman-Vario uncompensated','Kalman-Vario Speed Compensated',...
    'Speed Compensation','Absolute Speed 3d');

xlabel('Time / s');
ylabel('Altitude / m, Speed / m/s');
title('Variometer Speed Compensation');
