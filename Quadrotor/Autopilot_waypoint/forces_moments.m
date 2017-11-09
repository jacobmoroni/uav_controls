% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
%     e0      = x(7);
%     e1      = x(8);
%     e2      = x(9);
%     e3      = x(10);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_fr = delta(1);
    delta_fl = delta(2);
    delta_br = delta(3);
    delta_bl = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % convert to euler
    quat = euler_to_quat([phi, theta, psi]);
    e0 = quat(1);
    e1 = quat(2);
    e2 = quat(3);
    e3 = quat(4);
    % compute wind data in NED
    rotm_I_to_B = quat_to_rotm([e0; e1; e2; e3]);
    wind_inert = [w_ns; w_es; w_ds] + rotm_I_to_B'*[u_wg; v_wg; w_wg];
    w_n = wind_inert(1);
    w_e = wind_inert(2);
    w_d = wind_inert(3);
    
    % compute air data
    wind_body = rotm_I_to_B*wind_inert;
    u_r = u - wind_body(1);
    v_r = v - wind_body(2);
    w_r = w - wind_body(3);
    
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = 0;%atan(w_r/u_r);
    beta = 0;%asin(v_r/Va);
    
 
    
    % compute external forces and torques on aircraft
   
    F_and_T = [P.k1         ,P.k1          ,P.k1          ,P.k1;...
               -P.length*P.k1,P.length*P.k1,-P.length*P.k1,P.length*P.k1;...
               P.length*P.k1, P.length*P.k1,-P.length*P.k1,-P.length*P.k1;...
               -P.k2        ,P.k2          ,P.k2         ,-P.k2]...
               * [delta_fr; delta_fl; delta_br; delta_bl];
       
    Force = [0.0;0.0;F_and_T(1)];
    Torque = [F_and_T(2);F_and_T(3);F_and_T(4)];
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end



