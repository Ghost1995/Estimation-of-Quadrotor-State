function q = qaxisang2q(n,theta)
% Calculates the quaternion given axis and angle
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% n is [nx,ny,nz]'
% theta is measured in radians in anti-clockwise direction
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

q(1) = cos(theta/2);
q(2:4) = n.*sin(theta/2);
q = q';
end