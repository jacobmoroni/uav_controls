function [ output ] = states_12_to_13( x )
%states_12_to_13 Summary of this function goes here
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    
    euler = [phi;theta;psi];
    quat = euler_to_quat(euler);
    e0 = quat(1);
    e1 = quat(2);
    e2 = quat(3);
    e3 = quat(4);
    
    output(1) = pn;
    output(2) = pe;
    output(3) = pd;
    output(4) = u;
    output(5) = v;
    output(6) = w;
    output(7) = e0;
    output(8) = e1;
    output(9) = e2;
    output(10) = e3;
    output(11) = p;
    output(12) = q;
    output(13) = r;
    
    


end

