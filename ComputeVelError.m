function VelError = ComputeVelError(EstVel, TruthVel)
% Calculates the error between two velocities
% Code by: Nitin J. Sanket, nitinsan@terpmail.umd.edu
% Inputs:
% EstVel: Estimated velocity where each row is [x, y, z] at time t, i.e.,
% for N time steps the size will be Nx3
% TruthVel: Ground Truth velocity where each row is [x, y, z] at time t, i.e.,
% for N time steps the size will be Nx3
% Outputs:
% VelError: Absolute difference Error of size Nx3 for N time steps

VelError = abs(EstVel - TruthVel);
end
