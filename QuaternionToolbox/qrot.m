function vrot = qrot(q,v)
% Rotate a quaternion q according to vector v
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu
% http://www.mathworks.com/help/aerotbx/ug/quatrotate.html?requestedDomain=www.mathworks.com

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

vrot = [(1-2*q2^2-2*q3^2), 2*(q1*q2 + q0*q3),  2*(q1*q3-q0*q2);...
        2*(q1*q2-q0*q3),   (1-2*q1^2-2*q3^2),  2*(q2*q3+q0*q1);...
        2*(q1*q3+q0*q2),   2*(q2*q3-q0*q1),    (1-2*q1^2-2*q2^2);]'*v;
end