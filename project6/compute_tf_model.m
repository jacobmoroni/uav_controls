function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
a_phi1 = -(.5*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_p*P.b)/(2*P.Va0);
a_phi2 = .5*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_delta_a;

Va_trim = sqrt(P.u0^2+P.v0^2+P.w0^2);
 
a_theta1 = -.25*(P.rho*P.Va0^2*P.c*P.S_wing*P.C_m_q*P.c)/(P.Jy*P.Va0);
a_theta2 = -(P.rho*P.Va0^2 * P.c*P.S_wing*P.C_m_alpha)/(2*P.Jy);
a_theta3 =(P.rho*P.Va0^2 * P.c*P.S_wing*P.C_m_delta_e)/(2*P.Jy);

theta_trim = P.theta0;
alpha_trim = atan(P.w0/P.u0);

a_V1 = (P.rho*Va_trim*P.S_wing*(P.C_D_0+P.C_D_alpha*alpha_trim+...
    P.C_D_delta_e*u_trim(1)))/P.mass + (P.rho*P.S_prop*P.C_prop*Va_trim/P.mass);
a_V2 = (P.rho*P.S_prop*P.C_prop*P.k_motor^2*u_trim(4))/P.mass;
a_V3 = P.gravity*cos(theta_trim - alpha_trim);

a_beta1 = -(P.rho*P.Va0*P.S_wing*P.C_Y_beta)/(2*P.mass);
a_beta2 = (P.rho*P.Va0*P.S_wing*P.C_Y_delta_r)/(2*P.mass); 

    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

