% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0 0; 0 Yvdot Yrdot; 0 Nvdot Nrdot ];

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = inv(MRB + MA);

% linear damping
T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
D = -diag([Xu Yv Nr]);

% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

% input matrix
t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;

% surge desired veolcity
u_d = 7;


%% Problem 2b

% extracting sway-yaw subsystem
Minv_2dof = Minv(2:3,2:3);

% defining linearized matrices
CRB_lin = [0 0 0; 0 0 m*u_d; 0 0 m*xg*u_d];
CA_lin = [0 0 0; 0 0 -Xudot*u_d; 0 -Yvdot*u_d+Xudot*u_d -Yrdot*u_d];

% defining the ss-model
N = CRB_lin + CA_lin + D;
b_lin = 2*u_d * [-Y_delta; -N_delta];

A = -Minv_2dof * N(2:3,2:3);
B = Minv_2dof * b_lin;
C = [0 1];

[NUM,DEN] = ss2tf(A,B,C,0);
NUM;
DEN;

%% Problem 2c
num1 = NUM(2)/DEN(3);
num2 = NUM(3)/DEN(3);
den1 = DEN(1)/DEN(3)
den2 = DEN(2)/DEN(3);
den3 = DEN(3)/DEN(3);

K = num2;
T3 = num1/K;

roots(DEN);

%% Problem 2d
wb = 0.06; zeta = 1;
wn = 1/(sqrt(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2))) * wb;
Kp = T/K*wn^2
Kd = 2*1*wn*T/K - 1/K
Ki = wn/10 * Kp

