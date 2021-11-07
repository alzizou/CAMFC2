clear 
clc

N = 5; 
n = 2;
n0 = 1;

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

Q = 1.0e+2*eye(n); 

k1 = 1;
k2 = 1;
alpha = 0.01;

gama1 = 1e+3*diag([1e-6;1e-0]);
rho1 = 1e-0;

gama2 = 1e+0*diag([1e-6;1e-0]);
rho2 = 1e-0;

Observer_Gain = 10; 
Observer_Gain1 = 10;

Upsilon_m = 1;
Upsilon_M = Upsilon_m*ones(n,1);
X_m = 1;
chi = X_m*ones(n0,1);

r_form = 0.0;
Delta_form = [...
    +r_form 0;
    -2*r_form 0;
    -r_form 0;
    +2*r_form 0;
    +3*r_form 0];

Omega = [0 0;
    Delta_form];

%%
Jp = 1;
Bp = 0.01;
Mp = 0.1;
ge = 9.81;
lp = 0.1;

Disturb_Val = 1;
Disturb_Freq = 2;

%%
P_elems(1) = Simulink.BusElement;
P_elems(1).Name = 'p1';
P_elems(1).Dimensions = [n n];
P_elems(2) = Simulink.BusElement;
P_elems(2).Name = 'p2';
P_elems(2).Dimensions = [n n];
P_elems(3) = Simulink.BusElement;
P_elems(3).Name = 'p3';
P_elems(3).Dimensions = [n n];
P_elems(4) = Simulink.BusElement;
P_elems(4).Name = 'p4';
P_elems(4).Dimensions = [n n];
P_elems(5) = Simulink.BusElement;
P_elems(5).Name = 'p5';
P_elems(5).Dimensions = [n n];

P_bus = Simulink.Bus;
P_bus.Elements = P_elems;

P_InitVal = {zeros(n),zeros(n),zeros(n),zeros(n),zeros(n)};
P_struct = struct('data',P_InitVal);

%%
T_elems(1) = Simulink.BusElement;
T_elems(1).Name = 't1';
T_elems(1).Dimensions = [(N+1) n];
T_elems(2) = Simulink.BusElement;
T_elems(2).Name = 't2';
T_elems(2).Dimensions = [(N+1) n];
T_elems(3) = Simulink.BusElement;
T_elems(3).Name = 't3';
T_elems(3).Dimensions = [(N+1) n];
T_elems(4) = Simulink.BusElement;
T_elems(4).Name = 't4';
T_elems(4).Dimensions = [(N+1) n];
T_elems(5) = Simulink.BusElement;
T_elems(5).Name = 't5';
T_elems(5).Dimensions = [(N+1) n];

T_bus = Simulink.Bus;
T_bus.Elements = T_elems;

T_InitVal = {zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n)};
T_struct = struct('data',T_InitVal);

%%
epsilon_elems(1) = Simulink.BusElement;
epsilon_elems(1).Name = 'eps1';
epsilon_elems(1).Dimensions = [(N+1) n];
epsilon_elems(2) = Simulink.BusElement;
epsilon_elems(2).Name = 'eps2';
epsilon_elems(2).Dimensions = [(N+1) n];
epsilon_elems(3) = Simulink.BusElement;
epsilon_elems(3).Name = 'eps3';
epsilon_elems(3).Dimensions = [(N+1) n];
epsilon_elems(4) = Simulink.BusElement;
epsilon_elems(4).Name = 'eps4';
epsilon_elems(4).Dimensions = [(N+1) n];
epsilon_elems(5) = Simulink.BusElement;
epsilon_elems(5).Name = 'eps5';
epsilon_elems(5).Dimensions = [(N+1) n];

epsilon_bus = Simulink.Bus;
epsilon_bus.Elements = epsilon_elems;

epsilon_InitVal = {zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n)};
epsilon_struct = struct('data',epsilon_InitVal);

