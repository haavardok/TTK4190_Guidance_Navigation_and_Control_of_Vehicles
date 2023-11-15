% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           Håvard Olai Kopperstad
% Study program:    MITK
close all;
clear all;
% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v2'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.1;    % sampling time [s]
Ns = 63000;  % no. of samples

psi_ref = -110 * pi/180;% desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

% Current disturbance
Vc = 1;                 % Current speed (m/s)
beta_Vc = 45;           % Current angle (deg)

% Wind disturbance
Vw = 10;                % Wind speed (m/s)
beta_Vw = 135;          % Wind angle (deg)
rho_a = 1.247;          % Air density (kg/m3)
c_y = 0.95;             % Wind coefficient Y
c_n = 0.15;             % Wind coefficient N
L_oa = 161;             % Length overall
A_Lw = 10*L_oa;         % Lateral projected area

% initial states
eta_0 = [0 0 -110 * pi/180]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
x = [nu_0' eta_0' delta_0 n_0]';
xd = [0 0 0]';          % Reference model desired states: psi_d, psi_ddot, psi_dddot
e_int = 0;              % integral error
e_yint = 0;             % integral cross-track error

% intializing KF
x_prd = [x(6); x(3); 0];
P_prd = zeros(3);
Qd = diag([1,1]);       % Covariance matrix for the process noise
Rd = 1000;              % Covariance matrix for the measurement noise

% calculating the Kalman filter system matrices for estimating the yaw
% angle, yaw rate and the rudder bias
T_nomoto = -99.4713;    % Nomoto time constant
K_nomoto = -0.0049;     % Nomoto gain
[Ad,Bd,Cd,Dd,Ed] = discrete_kalman_matrices(T_nomoto,K_nomoto,h);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);        % table of simulation data
simdata2 = zeros(Ns+1,10);       % table of simulation data for crab and sideslip
wait_bar = waitbar(0, 'Starting');
for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1a) Add current disturbance here 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uc = Vc * cos(deg2rad(beta_Vc) - x(6));
    vc = Vc * sin(deg2rad(beta_Vc) - x(6));
    nu_c = [ uc vc 0 ]';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculating the relative velocity
    ur = x(1) - uc;              % relative surge velocity   
    vr = x(2) - vc;              % relatice sway velocity
    Ur = sqrt(ur^2+vr^2);        % relative speed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1c) Add wind disturbance here 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    u_w = Vw * cos(deg2rad(beta_Vw) - x(6));
    v_w = Vw * sin(deg2rad(beta_Vw) - x(6));
    u_rw = x(1) - u_w;
    v_rw = x(2) - v_w;
    Vrw = sqrt(u_rw^2 + v_rw^2);
    gamma_rw = -atan2(v_rw,u_rw);

    Ywind = 0;
    Nwind = 0;

    if (t > 200)        % add wind disturbance after 200 s
        Ywind = 0.5 * rho_a * Vrw^2 * c_y * sin(gamma_rw) * A_Lw;
        Nwind = 0.5 * rho_a * Vrw^2 * c_n * sin(2 * gamma_rw) * A_Lw * L_oa;
    end

    tau_wind = [0 Ywind Nwind]';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 4, 1b) Add the LOS guidance
    [xk1,yk1,xk,yk,last] = WP_selector(x);
    [e_y,pi_p] = crossTrackError(xk1, yk1, xk, yk, x);

    % Part 4, 2d) Add Integral LOS
%     lookahead = 10*161;     % delta (lookahead distance)
%     e_yint = (lookahead * e_y) / (lookahead^2 + (e_y + 0.1 * e_yint)^2);
%     
%     e_tol = pi;
%     if e_yint >= e_tol
%         e_yint = e_tol;
%     elseif e_yint <= -e_tol
%         e_yint = -e_tol;
%     end

    chi_d = LOS_guidance(e_y,pi_p,x,e_yint);
    psi_ref = chi_d;        % from problem description
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add a autopilot reference model here 
    % Define it as a function
    % check eq. (15.143) in (Fossen, 2021) for help
    %
    % The result should look like this:
    xd_dot = ref_model(xd,psi_ref);
    psi_d = xd(1);
    r_d = xd(2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     if t > 500
%        psi_ref = deg2rad(-20);
%     end
%     psi_d = psi_ref;
%     r_d = 0;
    u_d = U_ref;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add the heading controller here 
    % Define it as a function
    %
    % The result should look like this:
    e_psi = ssa(x(6) - psi_d);
    e_r = x(3) - r_d;
    e_int = e_int + h * e_psi;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 5, 2a) Add measurement noise here
    psi_m = x(6) + normrnd(0,0.5*pi/180);
    r_m = x(3) + normrnd(0,0.1*pi/180);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 5, 2b) Add KF here
    [x_pst,P_pst,x_prd,P_prd] = KF(x_prd,P_prd,Ad,Bd,Ed,Cd,Dd,Qd,Rd,psi_m,x(7));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 5, 2c) Using noisy measurements as feedback to heading autopilot
    e_psi = ssa(psi_m-psi_d);
    e_r = r_m-r_d;
    e_u = x(1)-u_d;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 5, 2d) Using state estimates as feedback to heading autopilot
    e_psi = ssa(x_pst(1)-psi_d);
    e_r = x_pst(2)-r_d;
    e_u = x(1)-u_d;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Implementing integrator anti-windup
    e_tol = pi;                  % This constant is drawn from the ass
    if e_int >= e_tol
        e_int = e_tol;
    elseif e_int <= -e_tol
        e_int = -e_tol;
    end
    
    delta_c = PID_heading(e_psi,e_r,e_int);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     delta_c = 0.1;              % rudder angle command (rad)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1e) Add open loop speed control here
    % Define it as a function
    %
    % The result should look like this:
    n_c = open_loop_speed_control(U_ref);  % propeller speed (rps)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     n_c = 10;                   

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1f) Replace the open loop speed controller, 
    % with a closed loop speed controller here 
    % Define it as a function
    %
    % The result should look like this:
    % n_c = closed_loop_speed_control(u_d,e_u,e_int_u);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % ship dynamics
    u = [delta_c n_c]';
    [xdot,u] = ship(x,u,nu_c,tau_wind,h);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d];
    simdata2(i,:) = [t ur vr Ur chi_d psi_m r_m x_pst'];
 
    % Euler integration
    x = euler2(xdot,x,h);
    xd = euler2(xd_dot,xd,h);

    % end simulation if last waypoint is reached
    if last == 1
        break;
    end

    waitbar(i/(Ns+1), wait_bar, sprintf('Progress: %d %%', floor(i/(Ns+1)*100)));
end
close(wait_bar);
simdata = simdata(1:i,:);
simdata2 = simdata2(1:i,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
u       = simdata(:,2);                 % m/s
v       = simdata(:,3);                 % m/s
r       = (180/pi) * simdata(:,4);      % deg/s
x       = simdata(:,5);                 % m
y       = simdata(:,6);                 % m
psi     = simdata(:,7);                 % rad
psi_deg = (180/pi) * psi;               % deg
delta_0 = (180/pi) * simdata(:,8);      % deg
n_0     = 60 * simdata(:,9);            % rpm
delta_c = (180/pi) * simdata(:,10);     % deg
n_c     = 60 * simdata(:,11);           % rpm
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s
psi_m   = (180/pi) * simdata2(:,6);     % deg
r_m     = (180/pi) * simdata2(:,7);     % deg
psi_hat = (180/pi) * simdata2(:,8);     % deg
r_hat   = (180/pi) * simdata2(:,9);     % deg/s
b_hat   = (180/pi) * simdata2(:,10);    % deg

%% Plotting states and path
figure(2)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');

figure(3)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
subplot(312)
plot(t,n_0,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
subplot(313)
plot(t,delta_0,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');

%% Plotting sideslip and crab
vr = simdata2(:,3);
Ur = simdata2(:,4);                         
Chi_d = simdata2(:,5)*180/pi;           % desired course (deg)
beta_c = atan(v(:,1)./u(:,1))*180/pi;   % crab angle (deg)
beta = asin(vr(:,1)./Ur(:,1))*180/pi;   % sideslip (deg)
chi = psi_deg + beta_c;                 % course (deg)

figure(4)
figure(gcf)
subplot(311)
hold on
plot(t,beta_c)
plot(t, beta,'--')
hold off
title('Crab angle, \beta_c, and sideslip angle, \beta'); xlabel('time (s)');
legend('\beta_c [deg]','\beta [deg]'); 
xlabel('time (s)');
ylabel('[deg]')
grid on

subplot(312)
hold on
plot(t,chi)
plot(t,Chi_d,'--')
hold off
grid
title('Course angle, \chi, and desired course angle, \chi_d')
legend('\chi [deg]','\chi_d [deg]'); 
ylabel('[deg]')
xlabel('time (s)');

subplot(313)
plot(t,psi_deg)
grid
title('Heading angle,\psi')
legend('\psi [deg]')
ylabel('[deg]')
xlabel('time (s)');

%% Plotting noisy measurements
figure(5)
hold on
plot(t, psi_m,'--')
plot(t,psi_deg)
hold off
title('Noisy measurements for yaw angle \psi'); xlabel('time (s)');
legend('\psi_m [deg]','\psi [deg]'); 
xlabel('time (s)');
ylabel('[deg]')
grid on

figure(6)
hold on
plot(t, r_m,'--')
plot(t,r)
hold off
title('Noisy measurements for yaw rate r'); xlabel('time (s)');
legend('r_m [deg]','r [deg]'); 
xlabel('time (s)');
ylabel('[deg/s]')
grid on

%% Plotting state estimates vs true states
figure(7)
figure(gcf)
subplot(311)
hold on
plot(t,psi_hat)
plot(t, psi_deg,'--')
hold off
title('Estimated yaw angle \psi_{hat} and true yaw angle \psi '); xlabel('time (s)');
legend('\psi_{hat} [deg]','\psi [deg]'); 
xlabel('time (s)');
ylabel('[deg]')
grid on

subplot(312)
hold on
plot(t,r_hat)
plot(t,r,'--')
hold off
grid
title('Estimated yaw rate r_{hat} and true yaw rate r')
legend('r_{hat} [deg/s]','r [deg/s]'); 
ylabel('[deg/s]')
xlabel('time (s)');

subplot(313)
plot(t,b_hat)
grid
title('Estimated Gaussian random-walk process driven by white noise, $b_{hat}$')
legend('b_{hat} [deg]')
ylabel('[deg]')
xlabel('time (s)');
ylim([-0.1,0.1])

%% Create objects for 3-D visualization 
% % Since we only simulate 3-DOF we need to construct zero arrays for the 
% % excluded dimensions, including height, roll and pitch
% z = zeros(length(x),1);
% phi = zeros(length(psi),1);
% theta = zeros(length(psi),1);
% 
% % create object 1: ship (ship1.mat)
% new_object('flypath3d_v2/ship1.mat',[x,y,z,phi,theta,psi],...
% 'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
% 'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
% 'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);
% 
% % Plot trajectories 
% flypath('flypath3d_v2/ship1.mat',...
% 'animate','on','step',500,...
% 'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
% 'font','Georgia','fontsize',12,...
% 'view',[-25 35],'window',[900 900],...
% 'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
% 'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
% 'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]);

% Plot path in pathplotter
pathplotter(x,y);