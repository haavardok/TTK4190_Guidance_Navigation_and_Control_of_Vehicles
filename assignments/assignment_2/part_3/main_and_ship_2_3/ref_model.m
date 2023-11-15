function xd_dot = ref_model(xd,psi_ref)
%REF_MODEL Summary of this function goes here
%   Detailed explanation goes here

w_ref = 0.03; zeta = 1;

A = [    0                  1                           0;
         0                  0                           1;
     -w_ref^3 -(w_ref^2 + 2*zeta*w_ref^2) -(2*zeta*w_ref + w_ref^2)];
B = [0 0 w_ref^3]';

xd_dot = A*xd + B*psi_ref;

end

