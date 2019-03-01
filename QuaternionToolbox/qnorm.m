function qnorm = qnorm(q)
% Normalizes the quaternion
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

qnorm = bsxfun(@rdivide,q,sqrt(sum(q.^2)));
end