function [vario, acc] = kalman_vario_XVA( in_alti, in_velo, in_acc)

T = 0.01;              % sampling time

A=[ 1 T T*T/2 0; 
    0 1 T 0; 
    0 0 1 0;
    0 0 0 1];      % system dynamics

C=[ 1 0 0 0
    0 1 0 0
    0 0 1 1];      % measurement

x = [ in_alti(1) 0 0 -9.81]'; % system start state

size = length(in_alti);

alti  = zeros( 1, size);
vario = zeros( 1, size);
acc   = zeros( 1, size);
acc_offset  = zeros( 1, size);

gain  = zeros( 1, size);

K = [
   0.014624427147059,   0.008213518721222,  -0.000020792258296
   0.018480417122749,   0.033924391681735,   0.006461499931904
   0.014990634309386,   0.067156008467552,   0.612880095929483
  -0.015011426567682,  -0.064284230720039,   0.006830749781626
   ];


for i = 1 : size
%    P = A * P * A' + Q;
%    K = P * C' / (C*P*C'+R);
    x = A * x;
    x = x + K * ( [in_alti(i) in_velo(i) in_acc(i)]' - C * x);
%    P = ( eye(4) - K * C) * P;
    
    alti(i)  = [1 0 0 0] * x;
    vario(i) = [0 1 0 0] * x;
    acc(i)   = [0 0 1 0] * x;
%    acc_offset(i) = [0 0 0 1] * x;

    gain(i)  = K(2,1);
end
