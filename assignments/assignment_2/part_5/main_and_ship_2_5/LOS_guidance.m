function chi_d = LOS_guidance(e_y,pi_p,x,e_yint)
% chi_d = LOS_guidance(e_y,pi_p) computes the desired course angle chi_d
% for a craft following a straight line between waypoints.
%
% Input:    e_y and pi_p, cross-track error expressed in NED
%           x(1) and x(2), absolute velocities in surge and sway
%
% Outputs:  chi_d, desired course angle
%

L = 161;                            % Length of ship (m)    
delta = 10 * L;                     % Lookahead distance
Kp = 1/delta;                       % Proportional gain Fossen (12.79)
Ki = 1;

% Part 4, 2c) Crab angle compensation
if (x(1)^2 + x(2)^2) <= 0
    % LOS guidance without crab angle compensation
    chi_d = pi_p - atan(Kp * e_y);  % Fossen (12.78)
else
    % LOS guidance with crab angle compensation
    chi_d = pi_p - atan(Kp * e_y) - asin(x(2) / sqrt(x(1)^2 + x(2)^2));      % Fossen (12.106)
end

% Part 4, 2d) Integral LOS
% chi_d = pi_p - atan(Kp*e_y + Ki * e_yint);  % Fossen (12.108)

end

