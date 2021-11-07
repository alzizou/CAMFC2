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

B = eye(n);
B_pseudo_inverse = ((B'*B)^(-1))*B';

R = 1e+0*eye(n);
Q = 1e+1*eye(n);
K = 1e+1*eye(n); 

k1 = 1;
k2 = 0.01;
alpha = 0.01;

gama1 = 1e+3*diag([1e-6*ones(n0,1);1e-0*ones(n0,1)]);
rho1 = 1e-0;
gama2 = 1e-1*diag([1e-6*ones(n0,1);1e-0*ones(n0,1)]);
rho2 = 1e-1;

%%

QR = 1e+0*eye(n);
BBT_inv = ((B'*B)^(-1))*B';
gamma_1 = 1e+1*diag([1e-6*ones(n0,1);1e-0*ones(n0,1)]);
rho_1 = 1e-2;
gamma_0 = 1e-2*diag([1e-6*ones(n0,1);1e-0*ones(n0,1)]); 
rho_0 = 1e-2;

%%

Observer_Gain = 10; 
Observer_Gain1 = 10;

U_dot0_m = 1;
U_dot0_M = U_dot0_m*ones(n0,1);
Upsilon_m = 1;
Upsilon_M = Upsilon_m*ones(n0,1);
X_m = 1;
X_M = X_m*ones(n0,1);

r_formX = 1.0;
r_formY = 1.0;
r_formZ = 1.0;
Beta = 0.1;

%%
Kf = 1e-5; % 6.11e-8; % each motor force constant (scalar)
Kt = 0.01 * Kf; %1.5e-9; % each motor torque constant (scalar)
Ka = 0.01; %*diag([1 1 1],0); % Aerodynamcs torque constant (matrix)
Kd = 0.01; %*diag([1 1 1],0); % Aerodynamics force constant (matrix)

Lq = 0.2; % Quadrotor each Arm Length (scalar)
J = 1.2416e-3 * diag([1 1 2],0); % Polar moment of inertia for the quadrotor (matrix)
M = 2; % Quadrotor total Mass (scalar)
ge = 9.81;  % the gravity acceleration = 9.81 m/s2.

Vr = 5;
VZr = 8;
Wr = 0.5;

run RECT_trajectory_generation.m

% RM = [Kf Kf Kf Kf;
%     -Lq*Kf 0 Lq*Kf 0;
%     0 -Lq*Kf 0 Lq*Kf;
%     Kt -Kt Kt -Kt];

RK = 0.1;
RL = 0.1;
RKf = 1e-5;
RM = RKf*[1 1 1 1;
    -RL 0 RL 0;
    0 -RL 0 RL;
    RK -RK RK -RK];

RM_1 = RM^(-1);

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
