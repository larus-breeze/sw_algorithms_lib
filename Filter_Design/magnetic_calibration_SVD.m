start =5*6000; 
stop = length(x);

format long;

new_sample_rate = 10; % Hz
old_sample_rate = 100;

mag_x = resample( x(114,start:stop),new_sample_rate, old_sample_rate ); 
mag_y = resample( x(115,start:stop),new_sample_rate, old_sample_rate ); 
mag_z = resample( x(116,start:stop),new_sample_rate, old_sample_rate ); 

err_x = resample( x(117,start:stop), new_sample_rate, old_sample_rate);
err_y = resample( x(118,start:stop), new_sample_rate, old_sample_rate);
err_z = resample( x(119,start:stop), new_sample_rate, old_sample_rate);

q1 = resample( x(53,start:stop),new_sample_rate, old_sample_rate );
q2 = resample( x(54,start:stop),new_sample_rate, old_sample_rate );
q3 = resample( x(55,start:stop),new_sample_rate, old_sample_rate );
q4 = resample( x(56,start:stop),new_sample_rate, old_sample_rate );

input = [mag_x' mag_y' mag_z' q1' q2' q3' q4'];

modelterms = ...
[   
     0     0     0     0     0     0     0;
     1     0     0     0     0     0     0; ...
     0     0     0     1     1     0     0; ...
     0     0     0     1     0     1     0; ...
     0     0     0     1     0     0     1; ...
     0     0     0     0     2     0     0; ...
     0     0     0     0     1     1     0; ...
     0     0     0     0     1     0     1; ...
     0     0     0     0     0     2     0; ...
     0     0     0     0     0     1     1; ...
     0     0     0     0     0     0     2];

poly_x = polyfitn( input, err_x, modelterms)
modelterms = ...
[   
     0     0     0     0     0     0     0;
     0     1     0     0     0     0     0; ...
     0     0     0     1     1     0     0; ...
     0     0     0     1     0     1     0; ...
     0     0     0     1     0     0     1; ...
     0     0     0     0     2     0     0; ...
     0     0     0     0     1     1     0; ...
     0     0     0     0     1     0     1; ...
     0     0     0     0     0     2     0; ...
     0     0     0     0     0     1     1; ...
     0     0     0     0     0     0     2];
poly_y = polyfitn( input, err_y, modelterms)
modelterms = ...
[   
     0     0     0     0     0     0     0;
     0     0     1     0     0     0     0; ...
     0     0     0     1     1     0     0; ...
     0     0     0     1     0     1     0; ...
     0     0     0     1     0     0     1; ...
     0     0     0     0     2     0     0; ...
     0     0     0     0     1     1     0; ...
     0     0     0     0     1     0     1; ...
     0     0     0     0     0     2     0; ...
     0     0     0     0     0     1     1; ...
     0     0     0     0     0     0     2];
poly_z = polyfitn( input, err_z, modelterms)

mag_err_predicted_x = polyvaln( poly_x, input)';
mag_err_predicted_y = polyvaln( poly_y, input)';
mag_err_predicted_z = polyvaln( poly_z, input)';

std_error_x = std( err_x)
std_error_x_corrected = std( err_x - mag_err_predicted_x)

std_error_y = std( err_y)
std_error_y_corrected = std( err_y - mag_err_predicted_y)

std_error_z = std( err_z)
std_error_z_corrected = std( err_z - mag_err_predicted_z)

std_error_mag_induction_abs = std( sqrt( ...
    (err_x - mag_err_predicted_x).*(err_x - mag_err_predicted_x)+ ...
    (err_y - mag_err_predicted_y).*(err_y - mag_err_predicted_y)+ ...
    (err_z - mag_err_predicted_z).*(err_z - mag_err_predicted_z)))

orintation_angle_error = sqrt( std_error_x * std_error_x + std_error_y * std_error_y +std_error_z * std_error_z)*180/pi

time = linspace( start / 6000, stop/6000, length( err_x));

a=subplot( 3,1,1);
plot( time, err_x,'Color','red');
grid
hold
plot( time, err_x - mag_err_predicted_x,'Color','black');
legend('Observed','Corrected')
title('Magnetic Field Error Front')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');
axis([0 max(time) -0.1 0.1]);

b=subplot( 3,1,2);
plot( time, err_y,'Color','red');
grid
hold
plot( time, err_y - mag_err_predicted_y,'Color','black');
legend('Observed','Corrected')
title('Magnetic Field Error Right')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');
axis([0 max(time) -0.1 0.1]);

c=subplot( 3,1,3);
plot( time, err_z,'Color','red');
grid
hold
plot( time, err_z - mag_err_predicted_z,'Color','black');
%plot( time, err_z - testz','Color','blue');
legend('Observed','Corrected')
title('Magnetic Field Error Down')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');
axis([0 max(time) -0.1 0.1]);

linkaxes([a b c],'x');

