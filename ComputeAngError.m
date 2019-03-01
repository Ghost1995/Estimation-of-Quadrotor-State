function AngError = ComputeAngError(EstAng, TruthAng)
% Calculates the error between two quaternions
% Code by: Nitin J. Sanket, nitinsan@terpmail.umd.edu
% Inputs:
% EstAng: Estimated Angle as a quaternion where each column is [QuatW, QuatX, QuatY, QuatZ]' at time t, i.e.,
% for N time steps the size will be 4xN
% TruthPos: Ground Truth Angle as a quaternion where each column is [QuatW, QuatX, QuatY, QuatZ]' at time t, i.e.,
% for N time steps the size will be 4xN
% Outputs:
% AngError: Norm Quaternion distance Error of size Nx1 for N time steps
AngError = qerr(EstAng, TruthAng);
end