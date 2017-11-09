% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    
    % simulate rate gyros (units are rad/sec)
    
    y_gyro_x = p+randn*P.sigma_gyro;
    y_gyro_y = q+randn*P.sigma_gyro;
    y_gyro_z = r+randn*P.sigma_gyro;

    % simulate accelerometers (units of g)
    
    y_accel_x = F_x/P.mass+P.gravity*sin(theta)+randn*P.sigma_accel;
    y_accel_y = F_y/P.mass-P.gravity*cos(theta)*sin(phi)+randn*P.sigma_accel;
    y_accel_z = F_z/P.mass-P.gravity*cos(theta)*cos(phi)+randn*P.sigma_accel;

    % simulate pressure sensors
    y_static_pres = P.rho*P.gravity*-pd+P.beta_abs_p+randn*P.sigma_abs_p;
    y_diff_pres = P.rho*Va^2/(2)+P.beta_diff_p+randn*P.sigma_diff_p;
    
    % simulate magnetometer
    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    st = sin(theta);
    cs = cos(psi);
    ss = sin(psi);
    
    R_v_to_b = [ct*cs, ct*ss, -st;...
        sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct;...
        cp*st*cs+sp*ss, cp*st*ss-sp*cs, cp*ct];
    
    mag_body = R_v_to_b*P.mag_inertia;
    mag_x = mag_body(1);
    mag_y = mag_body(2);
    mag_z = mag_body(3);

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



