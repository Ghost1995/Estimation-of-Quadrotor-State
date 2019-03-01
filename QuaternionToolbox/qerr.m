function Err = qerr(q1, q2)
% Calculates the error between two quaternions
% Quaternion is defined as [q0,q1,q2,q3]' where q = q0+iq1+jq2+kq3
% You can stack multiple quaternions as columns
% Code by: Nitin J. Sanket, nitinsan@terpmail.umd.edu
% Err is of size Nx1
Err = zeros(size(q1,2),1);
for count = 1:size(q1,2)
    Err(count) = norm([1,0,0,0]'-qmult(q1(:,count), qinv(q2(:,count))));
end
end