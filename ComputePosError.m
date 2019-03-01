function PosError = ComputePosError(EstPos, TruthPos)
% Calculates the error between two positions
% Code by: Nitin J. Sanket, nitinsan@terpmail.umd.edu
% Inputs:
% EstPos: Estimated Position where each row is [x, y, z] at time t, i.e.,
% for N time steps the size will be Nx3
% TruthPos: Ground Truth Position where each row is [x, y, z] at time t, i.e.,
% for N time steps the size will be Nx3
% Outputs:
% PosError: Euclidean distance Error of size Nx1 for N time steps

PosError = sqrt(sum((EstPos - TruthPos).^2,2));
end