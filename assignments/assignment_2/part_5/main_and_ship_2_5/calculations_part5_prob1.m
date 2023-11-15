clear all
clc

% Problem 1a: Kalman filter matrices in continuous time
T = -99.4713;                  % Nomoto time constant
K = -0.0049;                   % Nomoto gain

A = [0   1     0;
     0 -1/T  -K/T;
     0   0     0];

B = [0; K/T; 0];

C = [1; 0; 0];

E = [0 0;
     1 0;
     0 1];

% Problem 1b: Discretizing the system matrices using 1st order approximation (Euler integration)
h = 0.1;

Ad = eye(3) + A*h;
Bd = h*B;
Cd = C;
Ed = h*E;

% Problem 1c: Checking observability of the system
O = [C.'; C.'*A; C.'*A*A];
rank(O);

% Problem 2a: Generating noise





