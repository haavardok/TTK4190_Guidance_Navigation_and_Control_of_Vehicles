% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2018-08-15 Thor I. Fossen and Håkon H. Helgesen

%% USER INPUTS
h = 0.1;                     % sample time (s)
N  = 2500;                   % number of samples. Should be adjusted
kd = 400;                    % derivative gain
kp = 20;                     % proportional gain

% model parameters
m = 180;
r = 2;
I = m*r^2*eye(3);            % inertia matrix
I_inv = inv(I);
Kd = kd*eye(3);              % controller gain matrix

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi = -5*deg2rad;            % initial Euler angles
theta = 10*deg2rad;
psi = -20*deg2rad;

q = euler2q(phi,theta,psi);   % transform initial Euler angles to q

w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,14);        % memory allocation
ref_table = zeros(N+1,17);

%% FOR-END LOOP
for i = 1:N+1,
   t = (i-1)*h;                  % time

   phi_d = 0;                    % desired Euler angles
   theta_d = 15*cos(0.1*t)*deg2rad;
   psi_d = 10*sin(0.05*t)*deg2rad;
   Theta_d = [phi_d; theta_d; psi_d]';

   qd = euler2q(phi_d,theta_d,psi_d);   % transform qd to Euler angles
   qd_bar = [qd(1); -qd(2:4)];          % qd_bar (conjugate)
   q_tilde = quatprod(qd_bar,q);        % calculate the error
   epsilon_tilde = q_tilde(2:4);        % find the the error in epsilon

   phi_d_dot = 0;                % find Theta_d_dot
   theta_d_dot = -1.5*sin(0.1*t)*deg2rad;
   psi_d_dot = 0.5*cos(0.05*t)*deg2rad;
   Theta_d_dot = [phi_d_dot; theta_d_dot; psi_d_dot];

   w_d = Tzyx(Theta_d(1),Theta_d(2))\Theta_d_dot;
   w_tilde = w-w_d;

   tau = -Kd*w_tilde-kp*epsilon_tilde; % control law

   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics
   
   table(i,:) = [t q' phi theta psi w' tau'];  % store data in table
   ref_table(i,:) = [t qd' phi_d theta_d psi_d w' tau' Theta_d_dot'];
   
   q = q + h*q_dot;	             % Euler integration
   w = w + h*w_dot;
   
   q  = q/norm(q);               % unit quaternion normalization
end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11);  
tau     = table(:,12:14);
phi_d   = rad2deg*ref_table(:,6);
theta_d = rad2deg*ref_table(:,7);
psi_d   = rad2deg*ref_table(:,8);
w_dot   = rad2deg*ref_table(:,15:17);


figure (1); clf;
subplot(3,1,1);
title('Euler angles');
hold on;
plot(t, phi, 'b');
plot(t, phi_d, 'b--');
hold off;
grid on;
legend('\phi', '\phi_d');

subplot(3,1,2);
hold on;
plot(t, theta, 'r');
plot(t, theta_d, 'r--');
hold off;
grid on;
legend('\theta', '\theta_d');
ylabel('angle [deg]');

subplot(3,1,3);
hold on;
plot(t, psi, 'g');
plot(t, psi_d, 'g--');
hold off;
grid on;
legend('\psi', '\psi_d');
xlabel('time [s]'); 


figure (2); clf;
subplot(3,1,1);
title('Angular velocities');
hold on;
plot(t, w(:,1), 'b');
plot(t, w_dot(:,1), 'b--');
hold off;
grid on;
legend('p','p_d');

subplot(3,1,2);
hold on;
plot(t, w(:,2), 'r');
plot(t, w_dot(:,2), 'r--');
hold off;
grid on;
legend('q', 'q_d');
ylabel('angle [deg]');

subplot(3,1,3);
hold on;
plot(t, w(:,3), 'g');
plot(t, w_dot(:,3), 'g--');
hold off;
grid on;
legend('r','r_d');
xlabel('time [s]'); 


figure (3); clf;
hold on;
plot(t, tau(:,1), 'b');
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');