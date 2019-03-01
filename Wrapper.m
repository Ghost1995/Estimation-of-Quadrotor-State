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

% filename = 'DataMapping.mat';
% load('./Data/Mapping/DataMapping.mat');
% for i=1:length(DetAll)
%     LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Mapping/MappingFrames/' num2str(i) '.jpg']));
% end

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

filename = 'DataStraightLine.mat';
load('./Data/Straight Line/DataStraightLine.mat');
for i=1:length(DetAll)
    LeftImgs(:,:,i) = rgb2gray(imread(['./Data/Straight Line/StraightLineFrames/' num2str(i) '.jpg']));
end

load('./Data/CalibParams.mat');
load('LandMarksComputed.mat');

%% Localization using iSAM2
AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, LandMarksComputed, filename);

% % Plot the result
% figure
% subplot(1,3,1);
% hold on
% plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
% plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
% for i=1:4
%     plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
% end
% hold off
% legend({'Camera Poses','GroundTruth','April Tags'})
% xlabel('Distance in x-direction (m)')
% ylabel('Distance in y-direction (m)')
% view(2)
% 
% subplot(1,3,2);
% hold on
% plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
% plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
% for i=1:4
%     plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
% end
% hold off
% legend({'Camera Poses','GroundTruth','April Tags'})
% zlabel('Distance in z-direction (m)')
% ylabel('Distance in y-direction (m)')
% view(90,0)
% 
% subplot(1,3,3);
% hold on
% plot3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
% plot3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3),'g*');
% for i=1:4
%     plot3(LandMarksComputed(:,2*i),LandMarksComputed(:,2*i+1),zeros(size(LandMarksComputed,1),1),'r*');
% end
% hold off
% legend({'Camera Poses','GroundTruth','April Tags'})
% xlabel('Distance in x-direction (m)')
% zlabel('Distance in z-direction (m)')
% view(180,0)
% 
% supertitle('April Tag Locations and Camera Poses computed using iSAM2')

%% Estimate Velocity
dT = mean(abs(TLeftImgs(2:end) - TLeftImgs(1:end-1)));
Z = correctAltitude(AllPosesComputed, LandMarksComputed, TagSize, K);
V = velocityEstimator(LeftImgs, dT, K, Z);
% convert velocity to world frame
for i=1:length(DetAll)
    T = [QuatToRot(AllPosesComputed(i,4:7)),AllPosesComputed(i,1:3)'; 0 0 0 1];
    v = [T, zeros(4); zeros(4), T]*[V(i,1:3)'; 1; V(i,4:6)'; 1];
    V(i,:) = [v(1:3)', v(5:7)'];
end
VelError = ComputeVelError(V(:,1:3),VeliSAM2);

% % Plot the result
% figure
% subplot(1,3,1);
% hold on
% plot(V(:,1));
% plot(VeliSAM2(:,1));
% hold off
% legend({'Estimated Velocity','Actual Velocity'})
% xlabel('Frame Number')
% ylabel('Velocity in x-direction (m/s)')
% 
% subplot(1,3,2);
% hold on
% plot(V(:,2));
% plot(VeliSAM2(:,2));
% hold off
% legend({'Estimated Velocity','Actual Velocity'})
% xlabel('Frame Number')
% ylabel('Velocity in y-direction (m/s)')
% 
% subplot(1,3,3);
% hold on
% plot(V(:,3));
% plot(VeliSAM2(:,3));
% hold off
% legend({'Estimated Velocity','Actual Velocity'})
% xlabel('Frame Number')
% ylabel('Velocity in z-direction (m/s)')
% 
% supertitle('Velocity of the Quadrotor estimated using Camera Poses computed using iSAM2')
% 
% % Plot the result
% figure
% subplot(1,3,1);
% plot(VelError(:,1));
% legend({'Error in Velocity in x-direction'})
% xlabel('Frame Number')
% ylabel('Error (m)')
% 
% subplot(1,3,2);
% plot(VelError(:,2));
% legend({'Error in Velocity in y-direction'})
% xlabel('Frame Number')
% ylabel('Error (m)')
% 
% subplot(1,3,3);
% hold on
% plot(VelError(:,3));
% legend({'Error in Velocity in z-direction'})
% xlabel('Frame Number')
% ylabel('Error (m)')
% 
% supertitle('Error in Velocity Estimation using Camera Poses computed using iSAM2')