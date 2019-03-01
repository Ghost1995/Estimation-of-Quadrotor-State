function q = rv2q(rv,dt)
% Compute rotation vector from quaternion
% Code by: Nitin J. Sanket, nitinsan@seas.upenn.edu

if(nargin<2)
   dt = 1; 
end
alpha = norm(rv)*dt;
if(alpha==0)
    q = [1,0,0,0]';
    return;
end
ew = rv*dt./alpha;
q = [cos(alpha/2);ew.*sin(alpha/2)];
end