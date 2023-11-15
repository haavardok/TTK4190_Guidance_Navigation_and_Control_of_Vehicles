function [n_c] = open_loop_speed_control(U_ref)
%OPEN_LOOP_SPEED_CONTROL Compute desired propeller revolution nd

% ship parameters 
m = 17.0677e6;          % mass (kg)
rho = 1025;             % density of water (m/s^3)
Dia = 3.3;              % propeller diameter (m)
t = 0.05;               % thrust deduction number [0.05-0.2]
Ja = 0;                 % advance number of propeller
z = 4;                  % number of propeller blades
PD = 1.5;               % pitch/diameter ratio
AEAO = 0.65;            % blade area ratio

% added mass matrix
Xudot = -8.9830e5;

% linear damping
T1 = 20;
Xu = -(m-Xudot)/T1;

% compute propeller thrust and torque coefficients
[KT,KQ] = wageningen(Ja,PD,AEAO,z);

T_d = (U_ref * Xu) / (t-1);
n_c = sign(T_d) * sqrt(abs(T_d) / (rho*Dia^4*KT));

% n_c_squared = (Xu * U_ref) / ((t-1) * rho * Dia^4 * KT);
% n_c = sign(n_c_squared) * sqrt(abs(n_c_squared));

end

