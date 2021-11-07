clear 
clc

N = 4; 
n = 6;
m = 6;
n0 = 3;

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

%%
k1 = 50;
k2 = 1;
alpha = 0.1;
thresh_euler = 1e-4;

Q = 1e-0*eye(n0);
gama1 = 1e+2*diag(1e-0*ones(n0,1));
rho1 = 1e-0;
gama2 = 1e+0*diag(1e-0*ones(n0,1));
rho2 = 1e-0;

%%
Qv = 1e-0*eye(n0);
Rv = 1e-0*eye(n0);
Bv = eye(n0);
BvBT_inv = ((Bv'*Bv)^(-1))*Bv';
gamma_1v = 1e+2*diag(1e-0*ones(n0,1));
rho_1v = 1e-0;
gamma_0v = 1e-0*diag(1e-0*ones(n0,1)); 
rho_0v = 1e-0;

%%
QR = 1e-0*eye(n);
R = 1e-0*eye(n);
B = eye(n);
BBT_inv = ((B'*B)^(-1))*B';
gamma_1 = 1e+2*diag([1e-0*ones(n0,1);1e-0*ones(n0,1)]);
rho_1 = 1e-0;
gamma_0 = 1e+0*diag([1e-0*ones(n0,1);1e-0*ones(n0,1)]); 
rho_0 = 1e-0;

%%
Observer_Gain = 1; 
Observer_Gain1 = 1;

Upsilon_m = 0.1;
Upsilon_M = Upsilon_m*ones(n0,1);
X_m = 0.1;
chi = X_m*ones(n0,1);

r_formX = 1.0;
r_formY = 1.0;
r_formZ = 0.5;

Init_Pos_Dif = 0.0;

Beta = 0.2;

%%
Ka = 0.1; %*diag([1 1 1],0); % Aerodynamcs torque constant (matrix)
Kd = 0.1; %*diag([1 1 1],0); % Aerodynamics force constant (matrix)

Lq = 1; % Quadrotor each Arm Length (scalar)
J = 1.0e-3 * diag([1 1 2],0); % Polar moment of inertia for the quadrotor (matrix)
M = 2; % Quadrotor total Mass (scalar)
ge = 9.81;  % the gravity acceleration = 9.81 m/s2.

Vr = 5;
VZr = 8;
Wr = 0.5;

noise_amp = 0.1;
noise_frq = 1;

Vel_limit = 100;

run RECT_trajectory_generation.m

%%
P_elems(1) = Simulink.BusElement;
P_elems(1).Name = 'p1';
P_elems(1).Dimensions = [n0 n0];
P_elems(2) = Simulink.BusElement;
P_elems(2).Name = 'p2';
P_elems(2).Dimensions = [n0 n0];
P_elems(3) = Simulink.BusElement;
P_elems(3).Name = 'p3';
P_elems(3).Dimensions = [n0 n0];
P_elems(4) = Simulink.BusElement;
P_elems(4).Name = 'p4';
P_elems(4).Dimensions = [n0 n0];

P_bus = Simulink.Bus;
P_bus.Elements = P_elems;

P_InitVal = {zeros(n0),zeros(n0),zeros(n0),zeros(n0)};
P_struct = struct('data',P_InitVal);

%%
T_elems(1) = Simulink.BusElement;
T_elems(1).Name = 't1';
T_elems(1).Dimensions = [(N+1) n0];
T_elems(2) = Simulink.BusElement;
T_elems(2).Name = 't2';
T_elems(2).Dimensions = [(N+1) n0];
T_elems(3) = Simulink.BusElement;
T_elems(3).Name = 't3';
T_elems(3).Dimensions = [(N+1) n0];
T_elems(4) = Simulink.BusElement;
T_elems(4).Name = 't4';
T_elems(4).Dimensions = [(N+1) n0];

T_bus = Simulink.Bus;
T_bus.Elements = T_elems;

T_InitVal = {zeros((N+1),n0),zeros((N+1),n0),zeros((N+1),n0),zeros((N+1),n0)};
T_struct = struct('data',T_InitVal);

%%
epsilon_elems(1) = Simulink.BusElement;
epsilon_elems(1).Name = 'eps1';
epsilon_elems(1).Dimensions = [(N+1) n0];
epsilon_elems(2) = Simulink.BusElement;
epsilon_elems(2).Name = 'eps2';
epsilon_elems(2).Dimensions = [(N+1) n0];
epsilon_elems(3) = Simulink.BusElement;
epsilon_elems(3).Name = 'eps3';
epsilon_elems(3).Dimensions = [(N+1) n0];
epsilon_elems(4) = Simulink.BusElement;
epsilon_elems(4).Name = 'eps4';
epsilon_elems(4).Dimensions = [(N+1) n0];

epsilon_bus = Simulink.Bus;
epsilon_bus.Elements = epsilon_elems;

epsilon_InitVal = {zeros((N+1),n0),zeros((N+1),n0),zeros((N+1),n0),zeros((N+1),n0)};
epsilon_struct = struct('data',epsilon_InitVal);
