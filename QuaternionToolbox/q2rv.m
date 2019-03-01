function rv = q2rv(q)
% Computes quaternion from a rotation vector
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

% Angles
sinalpha = norm(q(2:4));
cosalpha = q(1);
% Numerically more stable than acos
alpha = atan2(sinalpha,cosalpha);
% alpha = acos(cosalpha);
if(sinalpha==0)
    rv = [0,0,0];
    return;
end
% Axes
e = transpose(q(2:4)/sinalpha);
rv = e*2*alpha;
end