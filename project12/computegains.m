%Compute Gains
%controller design

%%%%%%%%%%%%%%%%%%%%%%
% Roll Hold -- PHI
%%%%%%%%%%%%%%%%%%%%%%

% Tuning Params
e_phi_max = 60*pi/180; %Max Phi Step
zeta_phi = .7; %roll zeta

%TF delta_a to phi
[num,den] = tfdata(T_phi_delta_a,'v');
a_phi_2 = num(3);
a_phi_1 = den(2);

%max aileron command
delta_a_max = 45*pi/180;
% wn_phi = sqrt(abs(a_phi_2)*delta_a_max/e_phi_max);


%Gains for roll hold
P.kp_phi = delta_a_max/e_phi_max*sign(a_phi_2);

wn_phi = sqrt(P.kp_phi*a_phi_2);

P.kd_phi = (2*zeta_phi*wn_phi-a_phi_1)/a_phi_2;

%%%%%%%%%%%%%%%%%%%
%COURSE HOLD -- CHI
%%%%%%%%%%%%%%%%%%%

%tuning params
zeta_chi = .7;
bws = 20;

wn_chi = 1/bws*wn_phi;

% TF delta_a to phi
[num,den] = tfdata(T_chi_phi, 'v');
gVa = num(2);

P.kp_chi = 2*zeta_chi*wn_chi/gVa;
P.ki_chi = (wn_chi^2)/gVa;


% %gains for course hold
% P.kp_chi = 2*zeta_chi*wn_chi*P.Va0/P.g;
% P.ki_chi = wn_chi^2*P.Va_0/P.g;

%%%%%%%%%%%%%%%%%%%%%%
% PITCH COMMAND HOLD %
%%%%%%%%%%%%%%%%%%%%%%

%%%%TUNING PARAMS%%%%%

theta_step_max = 30*pi/180;
zeta_theta = .7; %Pitch
theta_max = 30*pi/180;

%TF delta_e to theta
[num,den] = tfdata(T_theta_delta_e, 'v');
a_theta1 = den(2);
a_theta2 = den(3);
a_theta3 = num(3);

P.kp_theta = ((theta_max)/(theta_step_max))*sign(a_theta3);

wn_theta = sqrt(a_theta2 +P.kp_theta*a_theta3);

P.kd_theta = -(2*zeta_theta*wn_theta - a_theta1)/a_theta3;

P.DC_theta = (P.kp_theta*a_theta3)/(a_theta2 + P.kp_theta*a_theta3);

%%%%%%%%%%%%%%
% TECS Gains %
%%%%%%%%%%%%%%
%TOTAL ENERGY CONTROL

%%THETA COMMAND FROM ENERGY DIFF%%
P.TECS_theta_kp = 1;
P.TECS_theta_kd = .75;
P.TECS_theta_ki = .5;

%%THROTTLE FROM TOTAL ENERGY%%
P.TECS_T_kp = 1;
P.TECS_T_kd = .75;
P.TECS_T_ki = .5;