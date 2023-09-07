clear
clc
syms R_33 m omega1 omega2 omega3 epsilon1 epsilon2 epsilon3 kp kd
%%
% Problem 1.2

R_33 = 2;
m = 180;
kd = 40;
kp = 2;

I_g = m*R_33^2*eye(3);
Tau = - 40*eye(3)*[omega1 omega2 omega3].' - 2*[epsilon1 epsilon2 epsilon3].';

B = [0 0 0;
     0 0 0;
     0 0 0;
     1/(m*R_33^2) 0 0;
     0 1/(m*R_33^2) 0;
     0 0 1/(m*R_33^2)];

A = [zeros(3) 0.5*eye(3);
     zeros(3) zeros(3)];

X = [epsilon1 epsilon2 epsilon3 omega1 omega2 omega3].';

X_dot = A*X + B*Tau;

K = [kp*eye(3) kd*eye(3)];

A_sys = (A-B*K); % X_dot = (A-BK)X

eig(A_sys)

%%
% Problem 1.3

kd = 40; kp = 2;

Epsilon = [epsilon1 epsilon2 epsilon3].';
Omega = [omega1 omega2 omega3].';

Kd = kd*eye(3);

Tau = -Kd*Omega-kp*Epsilon


