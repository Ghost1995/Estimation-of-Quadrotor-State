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
% https://drive.google.com/drive/folders/0B6NXn7BBGQf5MS0tUGR0Nno0Nk0
load('DataFastCircle.mat');
% load('DataMapping.mat');
% load('DataMountain.mat');
% load('DataSlowCircle.mat');
% load('DataSquare.mat');
% load('DataStraightLine.mat');
load('CalibParams.mat');

%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, [], TLeftImgs);

% Plot the result
figure
subplot(1,4,1);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseGTSAM(:,1),PoseGTSAM(:,2),PoseGTSAM(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
xlabel('Distance in x-direction (m)')
ylabel('Distance in y-direction (m)')
view(2)

subplot(1,4,2);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseGTSAM(:,1),PoseGTSAM(:,2),PoseGTSAM(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
zlabel('Distance in z-direction (m)')
ylabel('Distance in y-direction (m)')
view(90,0)

subplot(1,4,3);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseGTSAM(:,1),PoseGTSAM(:,2),PoseGTSAM(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
xlabel('Distance in x-direction (m)')
zlabel('Distance in z-direction (m)')
view(180,0)

% Compute Error
ValidIdxs = any(PoseGTSAM,2);
angError = zeros(size(PoseGTSAM,1),1);
posError = zeros(size(PoseGTSAM,1),1);
angError(ValidIdxs,1) = ComputeAngError(AllPosesComputed(ValidIdxs,4:7)',PoseGTSAM(ValidIdxs,4:7)');
posError(ValidIdxs,1) = ComputePosError(AllPosesComputed(ValidIdxs,1:3),PoseGTSAM(ValidIdxs,1:3));
subplot(1,4,4)
hold on
plot(angError)
plot(posError)
hold off
legend({'Angular Error','Position Error'})
xlabel('Frame Number')
ylabel ('Error')
view(2)
supertitle('April Tag Locations and Camera Poses computed using GTSAM')

%% Localization using iSAM2
AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, [], TLeftImgs, LandMarksComputed);
                                            
% Plot the result
figure
subplot(1,4,1);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
xlabel('Distance in x-direction (m)')
ylabel('Distance in y-direction (m)')
view(2)

subplot(1,4,2);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
zlabel('Distance in z-direction (m)')
ylabel('Distance in y-direction (m)')
view(90,0)

subplot(1,4,3);
hold on
plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
for i=1:4
    plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
end
hold off
legend({'Camera Poses','GroundTruth','April Tags'})
xlabel('Distance in x-direction (m)')
zlabel('Distance in z-direction (m)')
view(180,0)

% Compute Error
ValidIdxs = any(PoseiSAM2,2);
angError = zeros(size(PoseiSAM2,1),1);
posError = zeros(size(PoseiSAM2,1),1);
angError(ValidIdxs,1) = ComputeAngError(AllPosesComputed(ValidIdxs,4:7)',PoseiSAM2(ValidIdxs,4:7)');
posError(ValidIdxs,1) = ComputePosError(AllPosesComputed(ValidIdxs,1:3),PoseiSAM2(ValidIdxs,1:3));
subplot(1,4,4);
hold on
plot(angError)
plot(posError)
hold off
legend({'Angular Error','Position Error'})
xlabel('Frame Number')
ylabel ('Error')
view(2)
supertitle('April Tag Locations and Camera Poses computed using iSAM2')
