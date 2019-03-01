function qinv = qconj(q)
% Calculates the Conjugate quaternion
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu
qinv = [q(1,:); -q(2:4,:)];
end