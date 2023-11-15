function [Ad,Bd,Cd,Dd,Ed] = discrete_kalman_matrices(T,K,h)
% [Ad,Bd,Cd,Dd,Ed] = discrete_kalman_matrices(T,K) computes the discrete
% Kalman filter matrices for the system that estimates the yaw angle, yaw
% rate and the rudder bias for the ship. The differential equations are 
% based on the first order Nomoto ship model for the heading.
%
% Input:    T and K are the first order Nomoto ship model parameters.
%           h is the sampling time (s).
%
% Outputs:  [Ad,Bd,Cd,Dd,Ed] are the discrete Kalman filter matrices
%           for the specific LF ship model.
%

% Deriving the continuous time system matrices
A = [0   1     0;
     0 -1/T  -K/T;
     0   0     0];

B = [0; K/T; 0];

C = [1 0 0];

D = 0;

E = [0 0;
     1 0;
     0 1];

% Discretizing the system matrices using 1st order approximation (Euler integration)
Ad = eye(3) + A*h;
Bd = h*B;
Cd = C;
Dd = D;
Ed = h*E;

end
