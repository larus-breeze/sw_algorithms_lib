samplerate=100; % Hz

tau = 0.1;
[B,A]=butter( 2, 1 / samplerate / tau, 'low');

acceleration = -5; 
g = 9.81;

header_length = 2000;
acc = [ zeros(1,header_length) acceleration * ones(1,100) zeros(1,300) -acceleration * ones(1,100) zeros(1,header_length)];

acc = filter( B,A,acc);
vel = cumtrapz( acc) / samplerate;
pos = cumtrapz( vel) / samplerate;

initial_speed = 50; % m/s

if 1
    horiz_speed = initial_speed;
    horiz_acc = [ 0 diff( horiz_speed) * samplerate];
    nick_angle = 0;
    nick_rate = 0;
else
    horiz_speed = sqrt(  initial_speed * initial_speed - vel .* vel + 2 * g * pos);
    horiz_acc = [ 0 diff( horiz_speed) * samplerate];
    nick_angle = atan2( -vel, horiz_speed); % assuming nose pointing into direction of velocitiy 
    nick_rate = [ 0 diff( nick_angle) * samplerate];
end

vel = vel + ( - acc + g) .* horiz_speed / 100  * 0.5 / g; % friction
pos = cumtrapz( vel) / samplerate;

energy = 0.5*(vel.*vel + horiz_speed .* horiz_speed) - g * pos; % cross-check

pitot = 1.2255 / 2 * horiz_speed .* horiz_speed;

samples=length( pos);

inclination=65*pi/180;
mag_N=sin(inclination);
mag_D=cos(inclination);

% xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

sim_output=zeros( 37, samples);

sim_output(1,:) = horiz_acc .* cos( nick_angle) - (acc - 9.81) .* sin( nick_angle);
sim_output(3,:) = (horiz_acc .* sin( nick_angle)) + ((acc - 9.81) .* cos( nick_angle));

sim_output(5,:) = nick_rate;

sim_output(7,:) = mag_N;
sim_output(9,:) = mag_D;

sim_output(10,:)=pitot;
sim_output(11,:)=101325;

sim_output(16,:) = pos;

sim_output(17,:) = horiz_speed;
sim_output(19,:) = vel;

sim_output(20,:) = horiz_acc;
sim_output(22,:) = acc;

sim_output(25,:)=2; % replos N

sim_output(35,:)=1; % sat fix type

f=fopen('simout.f37','w');
fwrite( f,sim_output,'float');
fclose(f);
