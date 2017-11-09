% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    persistent nu_n;
    persistent nu_e;
    persistent nu_d;
    
    if t == 0
        nu_n = 0;
        nu_e = 0;
        nu_d = 0;
    end
    
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + nu_n;
    y_gps_e = pe + nu_e; 
    y_gps_h = pd + nu_d; 
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2+(Va*sin(psi)+we)^2)+randn*P.sigma_gps_Vg;
    y_gps_course = atan2(Va*sin(psi)+we,Va*cos(psi)+wn)+P.sigma_gps_chi;

    nu_n = exp(-P.k_gps*P.Ts_gps)*nu_n+randn*P.sigma_gps_n;
    nu_e = exp(-P.k_gps*P.Ts_gps)*nu_e+randn*P.sigma_gps_e;
    nu_d = exp(-P.k_gps*P.Ts_gps)*nu_d+randn*P.sigma_gps_d;
    
    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



