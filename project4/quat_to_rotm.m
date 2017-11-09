function [ rotm ] = quat_to_rotm( quat )
%This function converts a unit quaternion into a rotation matrix
rotm = zeros(3,3);
qw = quat(1);
qx = quat(2);
qy = quat(3);
qz = quat(4);

rotm(1,1) = 1- 2*qy^2 - 2*qz^2;
rotm(1,2) = 2*qx*qy - 2*qz*qw;
rotm(1,3) = 2*qx*qz + 2*qy*qw;
rotm(2,1) = 2*qx*qy + 2*qz*qw;
rotm(2,2) = 1- 2*qx^2 - 2*qz^2;
rotm(2,3) = 2*qy*qz - 2*qx*qw;
rotm(3,1) = 2*qx*qz - 2*qy*qw;
rotm(3,2) = 2*qy*qz + 2*qx*qw;
rotm(3,3) = 1- 2*qx^2 - 2*qy^2;


end
