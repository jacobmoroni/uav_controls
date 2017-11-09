%load aerosonde parameters
run aerosonde.m
P.gravity = 9.8;
   
%adjustable gain for quaternions
P.lambda = 100;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
P.Ts_estimator = P.Ts/10;
P.Ts_gps = 1.0;

% Dirty Derivative gain
P.tau = 0.05;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

%sensor sigmas
P.sigma_gyro = .13; %deg/s
P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;
P.sigma_accel = .0025; %g
P.beta_abs_p = .125; %kPa
P.sigma_abs_p = .01; %kPa
P.beta_diff_p = .02; %kPa
P.sigma_diff_p = .002; %kPa
P.sigma_gps_n = .21; %m
P.sigma_gps_e = .21; %m
P.sigma_gps_d = .40; %m
P.k_gps = 1/1100; %s
P.sigma_gps_Vg = 0.05; % m/s
P.sigma_gps_chi = P.sigma_gps_Vg/P.Va0; %m/s

P.mag_inclination = 65.7 * pi/180; % rad Pg. 132
P.mag_declination = 12.12 * pi/180; % rad Pg. 131

% Init mag field in inertial frame
mag_north = cos(P.mag_inclination)*cos(P.mag_declination);
mag_east = cos(P.mag_inclination)*sin(P.mag_declination);
mag_down = sin(P.mag_inclination);
P.mag_inertia = [mag_north;mag_east;mag_down];

% LPF Params
P.lpf_a = 50; % Cutoff freq for lpf
P.lpf_alpha = exp(-P.lpf_a*P.Ts);
P.lpf_a1 = 1.5; % Cutoff freq for lpf
P.lpf_alpha1 = exp(-P.lpf_a1*P.Ts);

%load gains
computegains;

%Guidance model update
P.b_chidot = 1.0;
P.b_Va = 1.0;
P.b_chi = 0.75;
P.b_hdot = 1.1;
P.b_h = 2.0;
P.gamma_max = 45*pi/180;

%path follow gains
P.Chi_inf = 60*pi/180;
P.k_path = 0.010;
P.k_orbit = 2.5;
 
% chapter 11 - path manager
P.phi_max = 37.5*pi/180;
% number of waypoints in data structure
P.size_waypoint_array = 120;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 2000;  % the city is of size (width)x(width)
building_height = 300;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
% P.pd0           = -h0;  % initial height of MAV
P.map = createWorld(city_width, building_height, num_blocks, street_width);

