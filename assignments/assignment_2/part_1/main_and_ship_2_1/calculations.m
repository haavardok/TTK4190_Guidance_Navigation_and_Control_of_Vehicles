%% Reset program
clear
clc

%% Values given from the problem description
L = 161;                % length (m)
B = 21.8;               % beam (m)
H = 15.8;               % heighy (m)
m = 17.0677e6;          % mass (kg)
g = 9.81;               % gravitational acceleration (m/s^2)
T = 4.74;               % draught (m)
I_zboat_CO = 2.1732e10; % yaw moment of inertia (kg/m^2)
rho_m = m/(L*B*H);      % density of ship (kg/m^3)
rho_w = 1025;           % density of water (kg/m^3)
r_bg_b = [-3.7 0 H/2]'; % vector from CG to CO in {b} frame

%% Problem 1a: Calculating I_z_CG,prism (element 3,3 of the Inertia dyadic p. 59 (3.22))

fun = @(x,y,z) x.^2 + y.^2;
xmin = -L/2;
xmax = L/2;
ymin = -B/2;
ymax = B/2;
zmin = -H/2;
zmax = H/2;
I_zprism_CG = rho_m * integral3(fun,xmin,xmax,ymin,ymax,zmin,zmax);

%% Problem 1b: Finding I_z_CO (the Parallell-axis Theorem p. 62 (3.37))

% Finding the expression for I_z_CO about CO for the rectangular ship
I_zprism_CO = I_zprism_CG + m * (r_bg_b(1).^2 + r_bg_b(2).^2);

% Calculating the ratio between the moments of inertia of the prism and the real ship
I_z_ratio = I_zprism_CO / I_zboat_CO;

%% Problem 1c: Finding M_RB_CO and C_RB_CO about the CO using r_bg_b

% Finding M_RB
M_RB = [     m            0     -m*r_bg_b(2);
             0            m      m*r_bg_b(1);
        -m*r_bg_b(2) m*r_bg_b(1) I_zprism_CO];

% Finding C_RB
% syms q p r Iz Iy Ix m x y z
% 
% r_bg_b = [x;y;z];
%
% H = [  eye(3)   Smtrx(r_bg_b).';
%       zeros(3)     eye(3)];
% 
% C_RB = H.' * [  0  -m*r  m*q   0      0      0;
%                m*r   0  -m*p   0      0      0;
%               -m*q  m*p   0    0      0      0;
%                 0    0    0    0     Iz*r  -Iy*q;
%                 0    0    0  -Iz*r    0     Ix*p;
%                 0    0    0   Iy*q  -Ix*p    0]   * H;
% 
% isequal(C_RB,-C_RB.')

% Finding C_RB_CO using linear-velocity independent parametrization, Fossen p. 68
syms m p q r x y z Ix Iy Iz Ixy Ixz Iyz

r_bg_b = [x; y; z];
nu2 = [p; q; r];

I_CG = [  Ix   -Ixy  -Ixz;
         -Ixy   Iy   -Iyz;
         -Ixz  -Iyz   Iz];

% Paralell axis theorem, Fossen p. 62
I_CO = I_CG - m*Smtrx(r_bg_b)^2;

C_RB_CO = [         m*Smtrx(nu2)           -m*Smtrx(nu2)*Smtrx(r_bg_b);    % Fossen p. 68
             m*Smtrx(r_bg_b)*Smtrx(nu2)         -Smtrx(I_CO*nu2)]


isequal(C_RB_CO, -C_RB_CO.')

%% Problem 2a: Computing volume displacement nabla

% Fossen p. 74 (4.11): m*g = rho*g*nabla
m = 17.0677e6;
nabla = m/rho_w;

%% Problem 2b: Waterplane area A_wp and expression Z_hs (hydrostatic force in heave DOF 3)

% Calculating Z_hs Fossen p. 75 (4.12): Z_hs = -rho*g*delta nabla(z^n)
% Fossen p. 87: nabla \approx A_wp * T (T = draught)
A_wp = nabla / T;

%% Problem 2c: Computing period T_3 in heave

% Fossen p. 87 (4.78)
T_3 = 2*pi*sqrt((2*T)/g)

%% Problem 2d: Finding GM_T and GM_L

% Fossen p. 81 (ex. 4.2)
CG = [80.5; 10.9; 7.9];

KG = CG(3);

KB = 1/3 * ((5*T)/2 - nabla/A_wp);

BG = KG - KB;

I_T = 1/12 * B^3 * L;

I_L = 1/12 * L^3 * B;

BM_T = I_T / nabla;
BM_L = I_L / nabla;

GM_T = BM_T - BG
GM_L = BM_L - BG


