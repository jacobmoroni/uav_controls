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
    e0      = x(7);
    e1      = x(8);
    e2      = x(9);
    e3      = x(10);
    p       = x(11);
    q       = x(12);
    r       = x(13);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
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
    alpha = atan(w_r/u_r);
    beta = asin(v_r/Va);
    
    % compute external forces and torques on aircraft
   C_D_alpha = P.C_D_p + ((P.C_L_0 + P.C_L_alpha*alpha)^2)/(pi*P.e*P.AR);
   sigma_alpha = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
   C_L_alpha = (1-sigma_alpha)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma_alpha*(2*sign(alpha)*(sin(alpha)^2)*cos(alpha));
    
   C_X_alpha = -C_D_alpha*cos(alpha) + C_L_alpha*sin(alpha);
   C_X_q_alpha = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
   C_X_delta_e_alpha = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
   C_Z_alpha = -C_D_alpha*sin(alpha) - C_L_alpha*cos(alpha);
   C_Z_q_alpha = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
   C_Z_delta_e_alpha = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
   
   
    Force(1) =  P.mass*P.gravity*2*(e1*e3 - e2*e0) + 0.5*P.rho*Va^2*P.S_wing*(C_X_alpha + C_X_q_alpha*P.c*q/(2*Va) + C_X_delta_e_alpha*delta_e)...
        +0.5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  P.mass*P.gravity*2*(e2*e3 + e1*e0) + 0.5*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b*p/(2*Va) + P.C_Y_r*P.b*r/(2*Va) + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  P.mass*P.gravity*(e3^2 + e0^2 - e1^2 - e2^2) + 0.5*P.rho*Va^2*P.S_wing*(C_Z_alpha + C_Z_q_alpha*P.c*w/(2*Va) + C_Z_delta_e_alpha*delta_e);
    
    Torque(1) = 0.5*P.rho*Va^2*P.S_wing*P.b*(P.C_ell_0 + P.C_ell_beta*beta + (P.C_ell_p*P.b*p/(2*Va)) + (P.C_ell_r*P.b*r/(2*Va)) + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r)...
        - P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = 0.5*P.rho*Va^2*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*alpha + (P.C_m_q*P.c*q/(2*Va)) + P.C_m_delta_e*delta_e);   
    Torque(3) = 0.5*P.rho*Va^2*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta + (P.C_n_p*P.b*p/(2*Va)) + (P.C_n_r*P.b*r/(2*Va)) + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



