function [phi_c,chi_int] = courseHold(chi,chi_c,Vg,chi_int,h)

% Add your controller gains and course hold PI controller here

% Constants
g = 9.81;
omega_n_phi = 0.5987;

% Design parameters
W_chi = 10;
zeta_chi = 0.63;

% Gains
omega_n_chi = omega_n_phi / W_chi;
k_p_chi = (2 * zeta_chi * omega_n_chi * Vg) / g;
k_i_chi = (omega_n_chi^2 * Vg) / g;

% Calculating PI gain
phi_c = k_p_chi * (chi_c-chi) + k_i_chi * chi_int;

% Euler's method: chi_int[k+1]
chi_int = chi_int + h * ssa(chi_c - chi);

end
