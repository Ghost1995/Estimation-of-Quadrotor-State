function qrotpaboutq = qrotpaboutq(p,q)
% Calculates the rotation of a point about a quaternion q
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% Point p is [px,py,pz]'
% p' = q[0,p]'invq
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

qrotpaboutq = qmult(qmult(q,qvec2q(p)),qinv(q));
end