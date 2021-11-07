clear 
clc

N = 4; 
n = 4;
n0 = 2; %nd

AN = [...
    0 1 0 0;
    1 0 1 0;
    0 1 0 1;
    0 0 1 0];

DN = [...
    1 0 0 0;
    0 2 0 0;
    0 0 2 0;
    0 0 0 1];

L = DN - AN;

BN = [...
    1 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0];

H = L + BN;

B = eye(n);
B_pseudo_inverse = ((B'*B)^(-1))*B';

R = 1e-0*eye(n);
Q = 1e-0*eye(n);
K = 1e-2*eye(n); 

k1 = 1;
k2 = 1;
alpha = 0.1;

gama1 = 1e4*diag([1e-6*ones(2,1);1e0*ones(2,1)]);
rho1 = 1e-1;

gama2 = 1e-2*diag([1e-6*ones(2,1);1e0*ones(2,1)]); 
rho2 = 1e-3;

Observer_Gain = 100; 
Observer_Gain1 = 100;

U_dot0_m = 10;
U_dot0_M = U_dot0_m*ones(n,1);
Upsilon_m = 10;
Upsilon_M = Upsilon_m*ones(n,1);

r_form = 0.0;
t_form = 0;
Delta_form = [...
    +r_form t_form 0 0;
    -r_form t_form 0 0;
    -0.5*r_form t_form 0 0;
    +0.5*r_form t_form 0 0];

chi = [0 0 0 0;
    Delta_form];

%% WMR
M = 1;
J = 0.01;
Cv = 0.1;
Cw = 0.1;
T0 = 10;
v0 = 10;
R0 = 0.1;
L0 = 0.1;

RM = (1/R0) * ...
    [1 1;
    L0 -L0];
RM_1 = RM^(-1);

run RECT_trajectory_generation.m

Thresh = 1e-4;

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

P_bus = Simulink.Bus;
P_bus.Elements = P_elems;

P_InitVal = {zeros(n),zeros(n),zeros(n),zeros(n)};
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

T_bus = Simulink.Bus;
T_bus.Elements = T_elems;

T_InitVal = {zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n)};
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

epsilon_bus = Simulink.Bus;
epsilon_bus.Elements = epsilon_elems;

epsilon_InitVal = {zeros((N+1),n),zeros((N+1),n),zeros((N+1),n),zeros((N+1),n)};
epsilon_struct = struct('data',epsilon_InitVal);

