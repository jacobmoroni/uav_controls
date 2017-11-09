function [ out ] = states_13_to_12( x )
    out(1)    = x(1);
    out(2)    = x(2);
    out(3)    = x(3);
    out(4)     = x(4);
    out(5)     = x(5);
    out(6)     = x(6);
    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);
    rpy = quat_to_euler([e0,e1,e2,e3]);
    out(7) = rpy(1);
    out(8) = rpy(2);
    out(9) = rpy(3);
    
    out(10)     = x(11);
    out(11)     = x(12);
    out(12)     = x(13);
end