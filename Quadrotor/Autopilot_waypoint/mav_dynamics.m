function [sys,x0,str,ts,simStateCompliance] = quad_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 13;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
% Init quaternions
quat = euler_to_quat([P.phi0, P.theta0, P.psi0]);
P.e00 = quat(1);
P.e10 = quat(2);
P.e20 = quat(3);
P.e30 = quat(4);

x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.e00;...
    P.e10;...
    P.e20;...
    P.e30;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);
    p     = x(11);
    q     = x(12);
    r     = x(13);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % norm of quaternion
    e_mag = sqrt(e0^2 + e1^2 + e2^2 + e3^2);
    % Lambda = adjustable gain
    
    quat_der = 0.5*[P.lambda*(1-e_mag^2), -p, -q, -r;...
                    p, P.lambda*(1-e_mag^2), r, -q;...
                    q, -r, P.lambda*(1-e_mag^2), p;...
                    r, q, -p, P.lambda*(1-e_mag^2)] * [e0; e1; e2; e3];
                            
                            
    pndot = (e1^2 + e0^2 - e2^2 - e3^2)*u + 2*(e1*e2 - e3*e0)*v + 2*(e1*e3 + e2*e0)*w;
    pedot = 2*(e1*e2 + e3*e0)*u + (e2^2 + e0^2 - e1^2 - e3^2)*v + 2*(e2*e3 - e1*e0)*w;
    pddot = 2*(e1*e3 - e2*e0)*u + 2*(e2*e3 + e1*e0)*v + (e3^2 + e0^2 - e1^2 - e2^2)*w;
    udot = r*v - q*w + fx/P.mass;
    vdot = p*w - r*u + fy/P.mass;
    wdot = q*u - p*v + fz/P.mass;
    e0dot = quat_der(1);
    e1dot = quat_der(2);
    e2dot = quat_der(3);
    e3dot = quat_der(4);
    pdot = P.gamma1*p*q - P.gamma2*q*r + P.gamma3*ell+P.gamma4*n;
    qdot = P.gamma5*p*r - P.gamma6*(p^2 - r^2) + m/P.Jy;
    rdot = P.gamma7*p*q - P.gamma1*q*r + P.gamma4*ell + P.gamma8*n;

sys = [pndot; pedot; pddot; udot; vdot; wdot; e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];
  
%   %persistent variables  
%   persistent mass
%   persistent Jx
%   persistent Jy
%   persistent Jz
%   persistent Jxz
%   
%   if t==0
%     alpha = 0.0;  % uncertainty parameter
%     mass = 13.5 * (1+2*alpha*rand-alpha);  % kg
%     Jx   = 0.8244* (1+2*alpha*rand-alpha);%kg-m^2
%     Jy   = 1.135* (1+2*alpha*rand-alpha); %kg-m^2
%     Jz   = 1.759* (1+2*alpha*rand-alpha); %kg-m^2
%     Jxz  = 0.1204* (1+2*alpha*rand-alpha);%kg-m^2
%     
%   end
%   
%     gamma = Jx*Jz - Jxz^2;
%     gamma1 = Jxz*(Jx-Jy+Jz)/gamma;
%     gamma2 = (Jz*(Jz-Jy)+Jxz^2)/gamma;
%     gamma3 = Jz/gamma;
%     gamma4 = Jxz/gamma;
%     gamma5 = (Jz-Jx)/Jy;
%     gamma6 = Jxz/Jy;
%     gamma7 = ((Jx-Jy)*Jx+Jxz^2)/gamma;
%     gamma8 = Jx/gamma;  
%   
%     pndot = cos(theta)*cos(psi)*u +(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v +...
%         (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
%     pedot = cos(theta)*sin(psi)*u +(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v +...
%         (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
%     pddot = -sin(theta)*u +sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
%     udot = r*v-q*w + fx/mass;
%     vdot = p*w-r*u + fy/mass;
%     wdot = q*u-p*v + fz/mass;
%     phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
%     thetadot = cos(phi)*q - sin(phi)*r;
%     psidot = sin(phi)/cos(theta)*q +cos(phi)/cos(theta)*r;
%     pdot = gamma1*p*q -gamma2*q*r + gamma3*ell + gamma4*n;
%     qdot = gamma5*p*r -gamma6*(p^2 - r^2) + 1/Jy*m;
%     rdot = gamma7*p*q -gamma1*q*r + gamma4*ell + gamma8*n;

% sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
