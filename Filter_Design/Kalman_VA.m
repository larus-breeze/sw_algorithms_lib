function [speed, acc] = Kalman_VA( in_velo, in_acc)

T = 0.01;              % sampling time

x = [ in_velo(1) in_acc(1)]'; % system start state

size = length(in_velo);

speed = zeros( 1, size);
acc   = zeros( 1, size);

K = [
   0.007715278655861,   0.003619628297543
   0.003701106163121,   0.004429968244752
   ];

A=[ 1 T ; 
    0 1 ]; 

C=[ 1 0; 
    0 1];

for i= 1 : size
%    P = A * P * A' + Q;
%    K = P * C' / (C*P*C'+R);
    x = A * x;
    x = x + K * ( [in_velo(i) in_acc(i)]' - C * x);
%    P = ( eye(4) - K * C) * P;
    
    speed(i) = [1 0] * x;
    acc(i)   = [0 1] * x;
%    acc_offset(i) = [0 0 0 1] * x;
end
