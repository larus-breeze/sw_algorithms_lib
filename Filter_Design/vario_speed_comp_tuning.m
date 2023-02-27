constant_energy = 1;
acceleration_based_speed_compensation = 1;
samplerate=100; % Hz

tau = 0.66; % varionmeter smoothing filter time  constant 
[B,A]=butter( 2, 1 / samplerate / tau, 'low'); % Vario Lowpass-Filter

acceleration = 10; 
g = 9.81;

% INS-reading from gravity, not from true acceleration,
% *automagically* removed by the Kalman-Vario-Observer
acceleleration_offset = - g; 

header_length = 100;
lift_duration = 300;
acceleration_time = 100;

vertical_acceleration = [ zeros(1,header_length) acceleration * ones(1, acceleration_time) zeros(1, lift_duration) -acceleration * ones(1,acceleration_time) zeros(1,header_length)];
vertical_acceleration = filter( B,A,vertical_acceleration);

vertical_speed = cumtrapz( vertical_acceleration) / samplerate;
altitude = cumtrapz( vertical_speed) / samplerate;

initial_speed = 50; % m/s
initial_altitude = 0;

if( constant_energy)
    energy_altitude = initial_speed ^2 / 2 / g + initial_altitude; % = energy / g := const !
    absolute_speed = sqrt( (energy_altitude - altitude) * 2 * g); % if energy = const
    horizontal_speed = sqrt( absolute_speed .^2 - vertical_speed .^2);
    horizontal_acceleration = [ 0 diff( horizontal_speed) * samplerate];
else % constant horizontal speed
    horizontal_speed = initial_speed * ones( 1, length( altitude));
    horizontal_acceleration = zeros( 1, length( altitude));
    absolute_speed = sqrt( horizontal_speed .^2 + vertical_speed .^2);
 end

[kalman_vario, kalman_acceleration] = kalman_vario_XVA( altitude, vertical_speed, vertical_acceleration + acceleleration_offset);

if( acceleration_based_speed_compensation)
    speed_compensation = (horizontal_speed .* horizontal_acceleration + kalman_vario .* kalman_acceleration) / g;
    simple_speed_compensation = (horizontal_speed .* horizontal_acceleration) / g;
else % velocity-differentiator-based speed compensation
    speed_compensation = [ 0 diff( absolute_speed .^2) * samplerate / 2 / g];
    simple_speed_compensation = [ 0 diff( horizontal_speed .^2) * samplerate / 2 / g];
end

speed_comp_vario = filter(B,A, kalman_vario + speed_compensation);
simple_speed_comp_vario = filter(B,A, kalman_vario + simple_speed_compensation);

absolute_speed = sqrt( horizontal_speed.^2 + vertical_speed.^2);

samples=length( altitude);
time=linspace(0,samples/samplerate,samples);


plot( time, altitude,'LineWidth',2.0)
hold
grid
plot( time, kalman_vario,'LineWidth',2.0);
plot( time, speed_comp_vario,'LineWidth',2.0);
plot( time, simple_speed_comp_vario,'LineWidth',2.0);
plot( time, speed_compensation,'LineWidth',2.0);
plot( time, absolute_speed,'LineWidth',2.0);

axis([0 samples/samplerate -15 70]);
legend('Altitude','Kalman-Vario uncompensated','Kalman-Vario Speed Compensated','Kalman-Vario Speed Compensated without vertical speed compensation',...
    'Speed Compensation', 'Absolute Speed 3d');

xlabel('Time / s');
ylabel('Altitude / m, Speed / m/s');
if( constant_energy)
    title('Variometer Reading, Test-case: Pull-up with constant energy');
else
    title('Variometer Reading, Test-case: Lift with constant horizontal speed');
end

