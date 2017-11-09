function output=Drone_Autopilot(uu,P)
    % process inputs
    NN = 0;
    pn       = uu(1+NN);  % inertial North position
    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    u        = uu(4+NN);  % airspeed
    v        = uu(5+NN);  % angle of attack
    w        = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % heading angle
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
    NN = NN+17;
    h_c      = uu(1+NN);  % commanded altitude (m)
    pn_c     = uu(2+NN);  % commanded north pos (m)
    pe_c     = uu(3+NN);  % commanded east pos (m)
    psi_c    = uu(4+NN);  % commanded heading angle
    NN = NN+4;
    t        = uu(1+NN);   % time
    
    %translate pn to x and pe to y
      pos_0 = [pn; pe];
      pos_0_c = [pn_c; pe_c];
%     x = pn*cos(psi) - pe*sin(psi);
%     y = pn*sin(psi) + pe*cos(psi);
%     x_c = pn_c*cos(psi) - pe_c*sin(psi);
%     y_c = pn_c*sin(psi) + pe_c*cos(psi);
    
    % set persistent flag to initialize integrators and differentiators at
    % the start of the simulation
    persistent flag
    if t<P.Ts
        flag = 1;
    else
        flag = 0;
    end
    %Lateral Dynamics --Psi            
    % loop theta_r to tau
    spin     = PD_psi(psi_c,psi,flag,P.kp_psi,P.kd_psi,...
                    P.taumax_psi,P.Ts,P.sigma,r,pos_0,pos_0_c,P.d_psi_max);
    
%     persistent x y x_c y_c
    tau_psi = spin(1);
    x = spin(2);
    y = spin(3);
    x_c = spin(4);
    y_c = spin(5);
    l_x = spin(6);
    l_y = spin(7);
    
    
    
    %longitudinal dynamics
    F_eq = (P.mass)*P.gravity/(cos(theta)*cos(phi));
    F_tilde = PID_h(h_c,h,flag,P.kp_h,P.ki_h,P.kd_h,...
        P.forcemax,P.Ts,P.sigma,w,F_eq,P.d_h_max);
    
    force = F_tilde + F_eq;
    
    % Lateral Dynamics--x
    
    % Outer loop x_c to theta_c
    theta_c = PID_x(x_c,x,flag,P.kp_x,P.ki_x,P.kd_x,...
                      P.Ts,P.sigma,u,P.thetamax,P.d_x_max,l_x);
                  
    % inner loop theta_r to tau
    tau_th     = PD_th(theta_c,theta,flag,P.kp_th,P.kd_th,...
                    P.taumax,P.Ts,P.sigma,p);
   
    % Lateral Dynamics--y
    
    % Outer loop y_c to phi_c
    phi_c = PID_y(y_c,y,flag,P.kp_y,P.ki_y,P.kd_y,...
                      P.Ts,P.sigma,v,P.phimax,P.d_y_max,l_y);
                  
    % inner loop theta_r to tau
    tau_phi     = PD_phi(phi_c,phi,flag,P.kp_phi,P.kd_phi,...
                    P.taumax,P.Ts,P.sigma,q);
                

                
    F_and_T = [force, tau_phi, tau_th, tau_psi];
    
    deltas = [P.k1         ,P.k1          ,P.k1          ,P.k1;...
             -P.length*P.k1,P.length*P.k1,-P.length*P.k1,P.length*P.k1;...
              P.length*P.k1, P.length*P.k1,-P.length*P.k1,-P.length*P.k1;...
             -P.k2        ,P.k2          ,P.k2         ,-P.k2]^-1 ...
              * F_and_T';
%     deltas = M^-1*F_and    
    delta_fr = deltas(1);
    delta_fl = deltas(2);
    delta_br = deltas(3);
    delta_bl = deltas(4); 
    
    x_command = [...
        pn_c;...                 % pn
        pe_c;...                 % pe
        h_c;...                  % h
        0;...                    % u
        0;...                    % v
        0;...                    % w
        phi_c;...                % phi
        theta_c;                 % theta
        psi_c;...                % psi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
    
    output = [delta_fr; delta_fl; delta_br; delta_bl; x_command];
                
       
end

%------------------------------------------------------------
% PID control for height (Longitudinal)
function u = PID_h(h_c,h,flag,kp,ki,kd,limit,Ts,sigma,w,F_eq, er_lim)
    % declare persistent variables
    persistent integrator
    persistent error_d1
    persistent hdot
    persistent h_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator  = 0;
        error_d1    = 0;
        hdot      = 0;
        h_d1      = h;
    end
    
    % compute the error
    error_unsat = h_c-h;
    error = sat(error_unsat,er_lim);
    
    % update derivative of phi
    hdot = (2*sigma-Ts)/(2*sigma+Ts)*hdot...
            + 2/(2*sigma+Ts)*(h-h_d1);
%     hdot = -w;
    % update delayed variables for next time through the loop
    h_d1 = h;
    error_d1 = error;
    
    % update integral of error
    integrator = integrator + (Ts/2)*(error+error_d1);
    % update delayed variables for next time through the loop
    error_d1 = error;

    % compute the pid control signal
    u_unsat = kp*error + ki*integrator -kd*hdot;
    u = sat_2(u_unsat,limit,F_eq);
end

%------------------------------------------------------------
% PID control for x
function theta_c = PID_x(x_c,x,flag,kp_x,ki_x,kd_x,Ts,sigma,u,limit,er_lim,l_x)
    % declare persistent variables
    persistent integrator_x
    persistent error_d1_x
    persistent xdot
    persistent x_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator_x  = 0;
        error_d1_x    = 0;
        xdot      = 0;
        x_d1      = x;
    end
    
    % compute the error
%     error_x_unsat = l_x;
    error_x_unsat = x_c-x;
    error_x = sat(error_x_unsat,er_lim);
    
    % update derivative of phi
%     xdot = (2*sigma-Ts)/(2*sigma+Ts)*xdot...
%              + 2/(2*sigma+Ts)*(x-x_d1);
    xdot = u;
    % update delayed variables for next time through the loop
    x_d1 = x;
    
    % update integral of error
    integrator_x = integrator_x + (Ts/2)*(error_x+error_d1_x);
    % update delayed variables for next time through the loop
    error_d1_x = error_x;

    % compute the pid control signal
    theta_c_unsat = kp_x*error_x + ki_x*integrator_x -kd_x*xdot;
    theta_c = sat(-theta_c_unsat,limit);
    
end


%------------------------------------------------------------
% PD control for angle theta
function u = PD_th(theta_c,theta,flag,kp_th,kd_th,limit,Ts,sigma,p)
    % declare persistent variables
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if flag==1
        thetadot    = 0;
        theta_d1    = theta;
    end
    
    % compute the error
    error = theta_c-theta;
    % update derivative of y
    thetadot = (2*sigma-Ts)/(2*sigma+Ts)*thetadot...
               + 2/(2*sigma+Ts)*(theta-theta_d1); 
%     thetadot = p;
    % update delayed variables for next time through the loop
    theta_d1 = theta;

    % compute the pid control signal
    u_unsat = kp_th*error - kd_th*thetadot;
    u = sat(u_unsat,limit);
    
end

%-----------------------------------------------------------------
%PID control for y
function u = PID_y(y_c,y,flag,kp_y,ki_y,kd_y,Ts,sigma,v,limit, er_lim,l_y)
    % declare persistent variables
    persistent integrator_y
    persistent error_d1_y
    persistent ydot
    persistent y_d1
    % reset persistent variables at start of simulation
    if flag==1
        integrator_y  = 0;
        error_d1_y    = 0;
        ydot      = 0;
        y_d1      = y;
    end
    
    % compute the error
%     error_unsat = l_y;
    error_y_unsat = y_c-y;
    error_y = sat(error_y_unsat, er_lim);
    
    % update derivative of phi
%     ydot = (2*sigma-Ts)/(2*sigma+Ts)*ydot...
%              + 2/(2*sigma+Ts)*(y-y_d1);
    ydot = v;
    % update delayed variables for next time through the loop
    y_d1 = y;
    
    % update integral of error
    integrator_y = integrator_y + (Ts/2)*(error_y+error_d1_y);
    % update delayed variables for next time through the loop
    error_d1_y = error_y;

    % compute the pid control signal
    u_unsat = kp_y*error_y + ki_y*integrator_y -kd_y*ydot;
    u = sat(u_unsat,limit);
    
end


%------------------------------------------------------------
% PD control for angle phi
function u = PD_phi(phi_c,phi,flag,kp_phi,kd_phi,limit,Ts,sigma,q)
    % declare persistent variables
    persistent phidot
    persistent phi_d1
    % reset persistent variables at start of simulation
    if flag==1
        phidot    = 0;
        phi_d1    = phi;
    end
    
    % compute the error
    error = phi_c-phi;
    % update derivative of y
    phidot = (2*sigma-Ts)/(2*sigma+Ts)*phidot...
               + 2/(2*sigma+Ts)*(phi-phi_d1);
%     phidot = q;%
    % update delayed variables for next time through the loop
    phi_d1 = phi;

    % compute the pid control signal
    u_unsat = kp_phi*error - kd_phi*phidot;
    u = sat(u_unsat,limit);
    
end

%------------------------------------------------------------
% PD control for angle psi
function out = PD_psi(psi_c,psi,flag,kp_psi,kd_psi,limit,Ts,sigma,r,pos_0,pos_0_c,er_lim)
    % declare persistent variables
    persistent psidot
    persistent psi_d1
    % reset persistent variables at start of simulation
    if flag==1
        psidot    = 0;
        psi_d1    = psi;
    end
    
    % compute the error
    error_unsat = psi_c-psi;
    error = sat(error_unsat,er_lim);
    % update derivative of y
%     psidot = (2*sigma-Ts)/(2*sigma+Ts)*psidot...
%                + 2/(2*sigma+Ts)*(psi-psi_d1);
    psidot = r;
    % update delayed variables for next time through the loop
    psi_d1 = psi;

    % compute the pid control signal
    u_unsat = kp_psi*error - kd_psi*psidot;
    u = u_unsat;%sat(u_unsat,limit);
    R = [cos(psi) -sin(psi);...
        sin(psi) cos(psi)];
    pos = R'*pos_0;
    pos_c = R'*pos_0_c;
    x = pos(1);
    y = pos(2);
    x_c = pos_c(1);
    y_c = pos_c(2);
    l = pos_c-pos;
    l_x = l(1);
    l_y = l(2);
    
    
    out = [u,x,y,x_c,y_c,l_x,l_y];
    
end


%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end

function out = sat_2(in,limit,F_eq)
    if     in > limit,      out = limit;
    elseif in < -F_eq+5,    out = -F_eq+5;
    else                    out = in;
    end
end

