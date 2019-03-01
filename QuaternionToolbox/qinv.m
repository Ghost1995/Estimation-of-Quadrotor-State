function qinv = qinv(q)
% Calculates the inverse of a quaternions
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% Assumes unit quaternion
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu
qinv = qconj(q);
% qinv = qnorm(q);
end