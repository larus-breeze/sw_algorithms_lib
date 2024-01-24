Poly_degree = 3;

if 0 % pick sector europe
    inc=inceur;
    dec=deceur;

    left = -15;
    right = 30;
    bottom = 25;
    top = 70;
    north = 49.7; % test bensheim
    east = 8.66;
end
if 0 % pick sector asia
    inc=incasia;
    dec=decasia;

    left = 30;
    right = 145;
    bottom = 10;
    top = 70;
    
    north = 40; % test beijing
    east = 123;
end
if 0 % pick sector australia
    inc=incaustralia;
    dec=decaustralia;

    left = 110;
    right = 155;
    bottom = -40;
    top = -10;
    
    north = -24; % test alice springs
    east = 134;
end

if 0 % pick sector usa
    inc=incusa;
    dec=decusa;

    left = -125;
    right = -70;
    bottom = 25;
    top = 70;
    
    north = 50; % test Los Alamos 
    east = -100;
end

if 0 % pick sector africa
    inc=incafrica;
    dec=decafrica;

    left = -20;
    right = 60;
    bottom = -35;
    top = 40;
    
    north = -22.5; % test windhoek: dec -12 W inc -63.75 
    east = 17;
end

if 0 % pick sector america (S)
    inc=incamerica;
    dec=decamerica;

    left = -80;
    right = -35;
    bottom = -40;
    top = 0;
    
    north = -20;
    east = -60;
end

if 0 % pick New Zealand
    inc=incnz;
    dec=decnz;

    left = 165;
    right = 179;
    bottom = -48;
    top = -34;
    
    north = -41.3; % wellington + 23.0 E -66.5
    east = 174.75;
end

if 1 % pick iceland
    inc=inciceland;
    dec=deciceland;

    left = -27;
    right = -10;
    bottom = 62;
    top = 67;
    
    north = 64.75; % center +10 W 75.75
    east = -18;
end

latitudes=top-bottom+1;
longitudes=right-left+1; % -180 .. 0 .. 179

declination=zeros( 1, latitudes * longitudes);
inclination=zeros( 1, latitudes * longitudes);
coordinates = zeros( 2, latitudes*longitudes);

for line=1:longitudes*latitudes
    coordinates(1,line) = dec(line,1); % latitude
    coordinates(2,line) = dec(line,2); % longitude

    declination( line)=dec(line,5);
    inclination( line)=inc(line,5);
end

poly_declination = polyfitn( coordinates', declination', Poly_degree);
poly_inclination = polyfitn( coordinates', inclination', Poly_degree);

[x,y]=meshgrid( bottom: top, left: right);

approx_declination = polyvaln( poly_declination, [x(:),y(:)]);

RMS_error_declination = std( approx_declination-declination')

approx_inclination = polyvaln( poly_inclination, [x(:),y(:)]);

RMS_error_inclination = std( approx_inclination-inclination')

figure(1)
surf( x, y, reshape( approx_declination, size(x)));
%hold
%plot3( coordinates(1,:),coordinates(2,:), declination, '+' )
xlabel('Latitude');
ylabel('Longitude');
zlabel('Mag. Declination / Degrees');
title('Polynomial Approximation')

figure(2)
surf( x, y, reshape( approx_declination-declination', size(x)));
xlabel('Latitude');
ylabel('Longitude');
zlabel('Mag. Declination Error / Degrees');
title('Polynomial Approximation')

figure(3)
surf( x, y, reshape( approx_inclination, size(x)));
xlabel('Latitude');
ylabel('Longitude');
zlabel('Mag. Inclination / Degrees');
title('Polynomial Approximation')

figure(4)
surf( x, y, reshape( approx_inclination-inclination', size(x)));
xlabel('Latitude');
ylabel('Longitude');
zlabel('Mag. Inclination Error / Degrees');
title('Polynomial Approximation')

format longE;
co=poly_declination.Coefficients
format short;

declination = ...
    co(10) + co(9) * east + co(8) * east * east + co(7) * east * east* east + co(6) * north + ...
    co(5) * north * east + co(4) * north * east * east + co(3) * north * north + ...
    co(2) * north * north *east + co(1) * north * north * north

format longE;
co=poly_inclination.Coefficients
format short;

inclination = ...
    co(10) + co(9) * east + co(8) * east * east + co(7) * east * east* east + co(6) * north + ...
    co(5) * north * east + co(4) * north * east * east + co(3) * north * north + ...
    co(2) * north * north *east + co(1) * north * north * north

