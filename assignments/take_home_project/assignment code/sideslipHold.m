function [delta_r_c, beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h)

% Add your controller gains and sideslip PI controller here

% Design parameters
delta_r_max = 30;
e_beta_max = 20;    % Asking chatGPT for normal value for F16
zeta_beta = 0.9;

% Gains
k_p_beta = delta_r_max / e_beta_max;    % Beard eq. 
omega_n_beta = (a_beta1 + a_beta2 * k_p_beta) / 2 * zeta_beta;
k_i_beta =  omega_n_beta^2 / a_beta2;

% Calculating PI gain
delta_r_c = -beta * k_p_beta + k_i_beta * beta_int;

% Euler's method: beta_int[k+1]
beta_int = beta_int + h * (-beta);

end