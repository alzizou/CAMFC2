clear
clc

N = 5; 
n = 2;
NN = 4;

AN = [...
    0 1 0 0 1;
    1 0 1 1 0;
    0 1 0 1 1;
    0 1 1 0 1;
    1 0 1 1 0];

DN = [...
    2 0 0 0 0;
    0 3 0 0 0;
    0 0 3 0 0;
    0 0 0 3 0;
    0 0 0 0 3];

L = DN - AN;

BN = [...
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 1 0 0;
    0 0 0 0 0;
    0 0 0 0 0];

H = L + BN;

q = (L + BN)^(-1)*ones(N,1);
p = 1./q;
P = diag(p);

F = 1.63;
c = 2.2;
k = 1;
lambda = 10; 
k_NN = 1;

%%
Jp = 1;
Bp = 0.01;
Mp = 0.1;
ge = 9.81;
lp = 0.1;

Disturb_Val = 1;
Disturb_Freq = 2;
