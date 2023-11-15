function delta_c = PID_heading(e_psi,e_r,e_int)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

wb = 0.06; zeta = 1;
T = 168.2782; K = 0.0745;
wn = 1/(sqrt(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2))) * wb;
Kp = T/K*wn^2;
Kd = 2*1*wn*T/K - 1/K;
Ki = wn/10 * Kp;

delta_c = -Kp*e_psi - Kd*e_r - Ki*e_int;

end

