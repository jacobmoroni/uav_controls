%airframe params
P.gravity = 9.81; %m/s^2

%adjustable gain for quaternions
P.lambda = 100; 

% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0; % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

%
P.Va0 = 25;
P.Ts = 0.01;

% Wind
P.wind_n    = 0;
P.wind_e    = 0;
P.wind_d    = 0;

% Dryden Gust Model Parameters
% Low alt, light turb
P.sigma_u = 1.06;
P.sigma_v = 1.06;
P.sigma_w = 0.7;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;

%physical parameters of airframe
P.mass = 1.280;%kg
P.Jx   = 0.005;%kg-m^2
P.Jy   = 0.005; %kg-m^2
P.Jz   = 0.5; %kg-m^2
P.length = .127; %m
P.k1 = 8.96914;
P.k2 = .05;%.5

%saturation 
P.d_psi_max = 15*pi/180;%to fly stable with movement, set this to about 5 to 10
P.d_x_max = 3;
P.d_y_max = 3;
P.d_h_max = 1;
P.sigma = .05;
P.forcemax = 5;
P.taumax = 1;
P.taumax_psi = 1;
P.thetamax = 15*pi/180;
P.phimax = 15*pi/180;
P.mu = 0.87; %momentum drag coefficient

%gains for latitudinal control
P.kp_h = 15;%15
P.kd_h = 14;%10;
P.ki_h = 2;%5

%gains for longitudinal control

P.kp_x = .19;%.2;
P.kd_x= .3;%.4;
P.ki_x = .01;%.03;%.005;%

P.kp_th = .1;
P.kd_th= .05;

P.kp_y = P.kp_x;
P.kd_y= P.kd_x;
P.ki_y = P.ki_x;

P.kp_phi = P.kp_th;
P.kd_phi= P.kd_th;

P.kp_psi = 1;
P.kd_psi= 1;

%error radius for waypoint following
P.way_rad = 1;
P.way_deg = 5*pi/180;


