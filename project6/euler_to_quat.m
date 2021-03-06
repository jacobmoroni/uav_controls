function [quaternion] = euler_to_quat(rpy)
%takes in Roll Pitch Yaw and converts to quaternions
phi   = rpy(1); %roll
theta = rpy(2); %pitch
psi   = rpy(3); %yaw

e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
e2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
e3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);
quaternion = [e0,e1,e2,e3];

end
