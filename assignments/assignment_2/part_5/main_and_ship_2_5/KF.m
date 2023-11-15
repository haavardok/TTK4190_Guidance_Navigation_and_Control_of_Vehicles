function [x_pst,P_pst,x_prd,P_prd] = KF(x_prd,P_prd,Ad,Bd,Ed,Cd,Dd,Qd,Rd,psi_meas,delta)
% [x_pst,P_pst,x_prd,P_prd] = KF(x_prd,P_prd,Ad,Bd,Ed,Cd,Qd,Rd,psi_meas,delta)
% computes the a priori and a posteriori state and covariance matrix
% estimates based on the previous a priori estimates and the discrete
% system matrices.
%
% Input:    x_prd: A priori state estimate
%           P_prd: A priori covariance matrix estimate
%           Ad,Bd,Cd,Ed: The discrete time system matrices
%           Qd,Rd: Covariance matrices for the process and measurement noises
%           psi_meas: Measured heading angle psi (noisy) (rad)
%           delta: The rudder angle (rad)
%
% Outputs:  x_pst: A posteriori state estimate
%           P_pst: A posteriori covariance matrix estimate
%           x_prd: New a priori state estimate
%           P_prd: New a priori covariance matrix estimate
%

n = length(Ad);
u = delta;

% KF gain: K[k]
K = P_prd * Cd' * inv( Cd * P_prd * Cd' + Rd );
IKC = eye(n) - K * Cd;

% Measurement: y[k]
y = psi_meas;

% Corrector: x_hat[k] and P_hat[k]
x_pst = x_prd + K * ( y - Cd * x_prd - Dd * u );
P_pst = IKC * P_prd * IKC' + K * Rd * K';

% Predictor: x_prd[k+1] and P_prd[k+1]
x_prd = Ad * x_pst + Bd * u;
P_prd = Ad * P_pst * Ad' + Ed * Qd * Ed';

end

