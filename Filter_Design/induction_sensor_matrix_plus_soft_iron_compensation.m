if( 1)
    start = 1
    stop=length(x)
else
    start = 105 * 6000; % choose ideal flight section
    stop  = 300 * 6000; 
end

new_sample_rate = 10; % Hz
old_sample_rate = 100;

if 1
 a_start = round( start * new_sample_rate / old_sample_rate);
 a_stop  = round( stop * new_sample_rate / old_sample_rate);
else
 a_start=1000;
 a_stop=length ( mag_err_predicted_x) -1000;
end

off_x = 0; % artificial offset injection
off_y = 0;
off_z = 0;

mag_x = resample( x(114,start:stop),new_sample_rate, old_sample_rate )- off_x; 
mag_y = resample( x(115,start:stop),new_sample_rate, old_sample_rate )- off_y; 
mag_z = resample( x(116,start:stop),new_sample_rate, old_sample_rate )- off_z; 

err_x = resample( x(117,start:stop), new_sample_rate, old_sample_rate)- off_x;
err_y = resample( x(118,start:stop), new_sample_rate, old_sample_rate)- off_y;
err_z = resample( x(119,start:stop), new_sample_rate, old_sample_rate)- off_z;

ideal_mag_x = mag_x - err_x;
ideal_mag_y = mag_y - err_y;
ideal_mag_z = mag_z - err_z;

q1 = resample( x(53,start:stop),new_sample_rate, old_sample_rate );
q2 = resample( x(54,start:stop),new_sample_rate, old_sample_rate );
q3 = resample( x(55,start:stop),new_sample_rate, old_sample_rate );
q4 = resample( x(56,start:stop),new_sample_rate, old_sample_rate );

% calculate sensor mapping matrix

input = [ mag_x' mag_y' mag_z'];

    modelterms = ...
[   
     0     0     0; ...
     1     0     0; ...
     0     1     0; ...
     0     0     1];

sensor_matrix_x = polyfitn( input, ideal_mag_x, modelterms)
induction_calibrated_x = polyvaln( sensor_matrix_x, input)';

sensor_matrix_y = polyfitn( input, ideal_mag_y, modelterms)
induction_calibrated_y = polyvaln( sensor_matrix_y, input)';

sensor_matrix_z = polyfitn( input, ideal_mag_z, modelterms)
induction_calibrated_z = polyvaln( sensor_matrix_z, input)';


% calculate soft iron correction terms

input = [ q1' q2' q3' q4'];

    modelterms = ...
[   
     0     0     0     0; ...
     2     0     0     0; ...
     1     1     0     0; ...
     1     0     1     0; ...
     1     0     0     1; ...
     0     2     0     0; ...
     0     1     1     0; ...
     0     1     0     1; ...
     0     0     2     0; ...
     0     0     1     1; ...
     0     0     0     2];

compensation_poly_x = polyfitn( input, induction_calibrated_x - ideal_mag_x, modelterms)
compensation_poly_y = polyfitn( input, induction_calibrated_y - ideal_mag_y, modelterms)
compensation_poly_z = polyfitn( input, induction_calibrated_z - ideal_mag_z, modelterms)

soft_iron_err_predicted_x = polyvaln( compensation_poly_x, input)';
soft_iron_err_predicted_y = polyvaln( compensation_poly_y, input)';
soft_iron_err_predicted_z = polyvaln( compensation_poly_z, input)';

induction_corrected_x = induction_calibrated_x - soft_iron_err_predicted_x;
induction_corrected_y = induction_calibrated_y - soft_iron_err_predicted_y;
induction_corrected_z = induction_calibrated_z - soft_iron_err_predicted_z;

residual_error_x = induction_corrected_x - ideal_mag_x;
residual_error_y = induction_corrected_y - ideal_mag_y;
residual_error_z = induction_corrected_z - ideal_mag_z;

std_error_x = std( err_x)
std_error_x_corrected = std( residual_error_x)

std_error_y = std( err_y)
std_error_y_corrected = std( residual_error_y)

std_error_z = std( err_z)
std_error_z_corrected = std( residual_error_z)

orientation_angle_error = sqrt( std_error_x * std_error_x + std_error_y * std_error_y +std_error_z * std_error_z)*180/pi

time = linspace( start / 6000, stop / 6000, length( err_x));

a=subplot( 3,1,1);
plot( time, err_x,'Color','red','LineWidth',2.0);
grid
hold
plot( time, induction_calibrated_x - ideal_mag_x,'Color','blue','LineWidth',2.0);
plot( time, induction_corrected_x - ideal_mag_x,'Color','black','LineWidth',2.0);
legend('Observed','Calibrated','Soft-Iron-Corrected')
title('Magnetic Field Error Front')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');
%axis([0 max(time) -0.1 0.1]);

b=subplot( 3,1,2);
plot( time, err_y,'Color','red','LineWidth',2.0);
grid
hold
plot( time, induction_calibrated_y - ideal_mag_y,'Color','blue','LineWidth',2.0);
plot( time, induction_corrected_y - ideal_mag_y,'Color','black','LineWidth',2.0);
legend('Observed','Calibrated','Soft-Iron-Corrected')
title('Magnetic Field Error Right')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');
%axis([0 max(time) -0.1 0.1]);

c=subplot( 3,1,3);
plot( time, err_z,'Color','red','LineWidth',2.0);
grid
hold
plot( time, induction_calibrated_z - ideal_mag_z,'Color','blue','LineWidth',2.0);
plot( time, induction_corrected_z - ideal_mag_z,'Color','black','LineWidth',2.0);
legend('Observed','Calibrated','Soft-Iron-Corrected')
title('Magnetic Field Error Down')
xlabel('Time / min.');
ylabel('Magn. Ind. Normalized');

linkaxes([a b c],'x');
