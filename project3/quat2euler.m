function [phi, theta, psi] = quat2euler(e)
e0 = e(1);
e1 = e(2);
e2 = e(3);
e3 = e(4);
phi = atan2(2*(e0*e1 + e2*e3),(e0^2+e3^2-e1^2-e2^2));
theta = asin(2*(e0*e2 - e1*e3));
psi = atan2(2*(e0*e3 + e1*e2),(e0^2+e1^2-e2^2-e3^2));
% euler = [phi,theta,psi];
end