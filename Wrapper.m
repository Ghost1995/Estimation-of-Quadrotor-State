%% Wrapper for P2Ph1 for CMSC828T Course at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
addpath('gtsam_toolbox');
addpath('QuaternionToolbox');

%% Load Data
% Download data from the following link: 
% https://drive.google.com/drive/folders/0B-6qEdcGmuD8djIxVVJtYmhBd0E

% filename = 'DataFastCircle.mat';
% load('./Data/Fast Circle/DataFastCircle.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Fast Circle/FastCircleFrames/' num2str(i) '.jpg']));
% end

filename = 'DataMapping.mat';
load('./Data/Mapping/DataMapping.mat');
for i=1:length(DetAll)
    LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Mapping/MappingFrames/' num2str(i) '.jpg']));
end

% filename = 'DataMountain.mat';
% load('./Data/Mountain/DataMountain.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Mountain/MountainFrames/' num2str(i) '.jpg']));
% end

% filename = 'DataSlowCircle.mat';
% load('./Data/Slow Circle/DataSlowCircle.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Slow Circle/SlowCircleFrames/' num2str(i) '.jpg']));
% end

% filename = 'DataSquare.mat';
% load('./Data/Square/DataSquare.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Square/SquareFrames/' num2str(i) '.jpg']));
% end

% filename = 'DataStraightLine.mat';
% load('./Data/Straight Line/DataStraightLine.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Straight Line/StraightLineFrames/' num2str(i) '.jpg']));
% end

load('./Data/CalibParams.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputedGTSAM, covariance] = SLAMusingGTSAM(DetAll, K, TagSize, filename);

%% Estimate Velocity
dT = [0;abs(TLeftImgs(2:end) - TLeftImgs(1:end-1))];
Z = correctAltitude(AllPosesComputedGTSAM, LandMarksComputed, TagSize, K);
V = velocityEstimator(LeftImgs, dT, K, Z);
% convert velocity to world frame
VelComputedGTSAM = zeros(length(DetAll),3);
for i=1:length(DetAll)
    VelComputedGTSAM(i,:) = QuatToRot(AllPosesComputedGTSAM(i,4:7))*V(i,1:3)' + AllPosesComputedGTSAM(i,1:3)';
end

%% Localization using iSAM2
AllPosesComputediSAM = LocalizationUsingiSAM2(DetAll, K, TagSize, LandMarksComputed, filename);

%% Estimate Velocity
dT = [0;abs(TLeftImgs(2:end) - TLeftImgs(1:end-1))];
Z = correctAltitude(AllPosesComputediSAM, LandMarksComputed, TagSize, K);
V = velocityEstimator(LeftImgs, dT, K, Z);
% convert velocity to world frame
VelComputediSAM = zeros(length(DetAll),3);
for i=1:length(DetAll)
    VelComputediSAM(i,:) = QuatToRot(AllPosesComputediSAM(i,4:7))*V(i,1:3)' + AllPosesComputediSAM(i,1:3)';
end
VelError = ComputeVelError(VelComputediSAM,VeliSAM2);
covariance = [covariance, zeros(6,3); zeros(3,6), cov(VelError)];

%% Extended Kalman Filter
PoseEKF = extendedKalmanFilter(AllPosesComputediSAM, VelComputediSAM, IMU, TLeftImgs, covariance);

% Plot the result (Position)
figure
subplot(1,3,1);
hold on
plot(AllPosesComputediSAM(:,1));
plot(PoseEKF(:,1));
plot(PoseGTSAM(:,1));
plot(AllPosesComputedGTSAM(:,1));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Position in x-direction (m)')

subplot(1,3,2);
hold on
plot(AllPosesComputediSAM(:,2));
plot(PoseEKF(:,2));
plot(PoseGTSAM(:,2));
plot(AllPosesComputedGTSAM(:,2));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Position in y-direction (m)')

subplot(1,3,3);
hold on
plot(AllPosesComputediSAM(:,3));
plot(PoseEKF(:,3));
plot(PoseGTSAM(:,3));
plot(AllPosesComputedGTSAM(:,3));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Position in z-direction (m)')

supertitle('Position of the Quadrotor')
    
OrientationComputediSAM = zeros(length(DetAll),3);
for i=1:length(DetAll)
    OrientationComputediSAM(i,:) = RotToRPY_ZXY(QuatToRot(AllPosesComputediSAM(i,4:7)));
end
OrientationGTSAM = zeros(length(DetAll),3);
for i=1:length(DetAll)
    OrientationGTSAM(i,:) = RotToRPY_ZXY(QuatToRot(PoseGTSAM(i,4:7)));
end
OrientationComputedGTSAM = zeros(length(DetAll),3);
for i=1:length(DetAll)
    OrientationComputedGTSAM(i,:) = RotToRPY_ZXY(QuatToRot(AllPosesComputedGTSAM(i,4:7)));
end
% Plot the result (Orientation)
figure
subplot(1,3,1);
hold on
plot(OrientationComputediSAM(:,1));
plot(PoseEKF(:,4));
plot(OrientationGTSAM(:,1));
plot(OrientationComputedGTSAM(:,1));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Orientation about x-direction (rad)')
    
subplot(1,3,2);
hold on
plot(OrientationComputediSAM(:,2));
plot(PoseEKF(:,5));
plot(OrientationGTSAM(:,2));
plot(OrientationComputedGTSAM(:,2));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Orientation about y-direction (rad)')

subplot(1,3,3);
hold on
plot(OrientationComputediSAM(:,3));
plot(PoseEKF(:,6));
plot(OrientationGTSAM(:,3));
plot(OrientationComputedGTSAM(:,3));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Orientation about z-direction (rad)')
    
supertitle('Orientation of the Quadrotor')
    
% Plot the result (Velocity)
figure
subplot(1,3,1);
hold on
plot(VelComputediSAM(:,1));
plot(PoseEKF(:,7));
plot(VelGTSAM(:,1));
plot(VelComputedGTSAM(:,1));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Velocity in x-direction (m/s)')
    
subplot(1,3,2);
hold on
plot(VelComputediSAM(:,2));
plot(PoseEKF(:,8));
plot(VelGTSAM(:,2));
plot(VelComputedGTSAM(:,2));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Velocity in y-direction (m/s)')

subplot(1,3,3);
hold on
plot(VelComputediSAM(:,3));
plot(PoseEKF(:,9));
plot(VelGTSAM(:,3));
plot(VelComputedGTSAM(:,3));
hold off
legend({'iSAM2 Output','EKF Output','GTSAM Ground Truth','GTSAM Output'})
xlabel('Frame Number')
ylabel('Velocity in z-direction (m/s)')
    
supertitle('Velocity of the Quadrotor')