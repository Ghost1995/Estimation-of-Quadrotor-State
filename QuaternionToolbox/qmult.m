function qmult = qmult(q1,q2)
% Calculates the product of two quaternions
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

qmult(1) = sum(q1.*[q2(1),-q2(2),-q2(3),-q2(4)]');
qmult(2) = sum(q1.*[q2(2),q2(1),q2(4),-q2(3)]');
qmult(3) = sum(q1.*[q2(3),-q2(4),q2(1),q2(2)]');
qmult(4) = sum(q1.*[q2(4),q2(3),-q2(2),q2(1)]');
qmult = qmult';
end