function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 3;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 3;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            %full throttle
            %constant pitch theta_c
        case 2,  % climb zone
             %full throttle
             %regulate airspeed with pitch
        case 3, % descend zone
            %zero throttle
            %regulate airspeed with pitch
        case 4, % altitude hold zone
            %regulate altitude with pitch
            %regulate airspeed with throttle
    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    Ek_nom = .5*P.mass*P.Va0^2;
    
    Ek = .5*P.mass*Va^2; %Kinetic Energy
    Ek_tilde = .5*P.mass*(Va_c^2-Va^2); 
    
    Ep = P.mass*P.gravity*h;%Potential Energy
    Ep_tilde = P.mass*P.gravity*(h_c-h);
    
    Et = Ek + Ep; %Total Energy
    Et_tilde = (Ek_tilde + Ep_tilde)/Ek_nom;
    
    Ed = Ep - Ek; %Energy Difference
    Ed_tilde = (Ep_tilde - Ek_tilde)/Ek_nom;
    
    if t==0
        delta_t = TECS_T(Et_tilde, 1, P);
        theta_c = TECS_theta(Ed_tilde, 1, P);
    else
        delta_t = TECS_T(Et_tilde, 0, P);
        theta_c = TECS_theta(Ed_tilde, 0, P);
    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
%     
%     delta_e = 0;
%     delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, P)
    % compute the error
    error = phi_c-phi;

    % compute the pd control signal
    u_unsat = P.kp_phi*error - P.kd_phi*p;
    delta_a = sat(u_unsat,45*pi/180,-45*pi/180);
end

function phi_c  = course_hold(chi_c, chi, r, flag, P)

% declare persistent variables
    persistent integrator
    persistent error_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator  = 0;
        error_d1    = 0;
    end
    
    % compute the error
    error = chi_c-chi;
    % update integral of error

    integrator = integrator + (P.Ts/2)*(error+error_d1);

    % compute the pd control signal
    phi_c_unsat = P.kp_chi*error + P.ki_chi*integrator;
    phi_c = sat(phi_c_unsat,45*pi/180,-45*pi/180);
    
    % integrator anti-windup
    if P.ki_chi~=0
        integrator = integrator + P.Ts/P.ki_chi*(phi_c-phi_c_unsat);
    end
    
    %update error
    error_d1 = error;
end

function delta_e = pitch_hold(theta_c, theta, q, P)
    % Pg. 107
    % PD Control
    
    % Current Error
    error = theta_c - theta;
%     disp(error)
    
    % Proportional
    up = P.kp_theta*error;
    
    % Derivative
    ud = P.kd_theta * q;
    
    % PD Control
    delta_e = sat(up+ud, 45*pi/180, -45*pi/180);
    
end

function delta_t = TECS_T(Et_tilde, flag, P)
    persistent integrator;
    persistent differentiator;
    persistent error_d1;
    
    if flag == 1
        integrator = 0;
        differentiator = 0;
        error_d1 = 0;
    end
    
    %current error
    error = Et_tilde;
    
    % Integrator
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    
    % Differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator +...
        2/(2*P.tau+P.Ts)*(error - error_d1);
    
     % Proportional
    up = P.TECS_T_kp*error;
    
    % Derivative
    ud = P.TECS_T_kd*differentiator;
    
    % Integral
    ui = P.TECS_T_ki * integrator;
    
    % PID Control
    disp('delta_t_unsat')
    delta_t_unsat = up + ud + ui;
    delta_t = sat(delta_t_unsat, 1, 0);
    
    % Anti-windup
    if P.TECS_T_ki ~= 0
        integrator = integrator + (P.Ts/P.TECS_T_ki)*(delta_t - delta_t_unsat);
    end
    
    % Update error
    error_d1 = error;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TECS Theta
% theta_c from energy diff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function theta_c = TECS_theta(Ed_tilde, init, P)
    % Slides
    % PID Control
    % kp on Ed_tilde + ki on Ed_tilde + kd on Ed_tilde
    persistent integrator;
    persistent differentiator;
    persistent error_d1;
    
    if init
        integrator = 0;
        error_d1 = 0;
        differentiator = 0;
    end
    
    % Current Error
    error = Ed_tilde;
    
    % Integrator
    integrator = integrator + (P.Ts/2)*(error+error_d1);
    
    % Differentiator
    differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator +...
        2/(2*P.tau+P.Ts)*(error - error_d1);
    
    % Proportional
    up = P.TECS_theta_kp*error;
    
    % Derivative
    ud = P.TECS_theta_kd*differentiator;
    
    % Integral
    ui = P.TECS_theta_ki * integrator;
    
    % PID Control
    disp('theta_c_unsat')
    theta_c_unsat = up + ud + ui;
    theta_c = sat(theta_c_unsat, 45*pi/180, -45*pi/180);
    
    % Anti-windup
    if P.TECS_theta_ki ~= 0
        integrator = integrator + (P.Ts/P.TECS_theta_ki)*(theta_c - theta_c_unsat);
    end
    
    % Update error
    error_d1 = error;
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end
  
 