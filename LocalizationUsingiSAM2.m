function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, LandMarksComputed, filename)
% For Input and Output specifications refer to the project pdf

import gtsam.*

% Create graph container and add factors to it
graph = NonlinearFactorGraph;

% define noise parameters
if all(ismember(filename,'DataFastCircle.mat'))
    noise1 = noiseModel.Diagonal.Sigmas([0.001; 0.001; 0.001]);
    noise2 = noiseModel.Diagonal.Sigmas([10; 10; 1; 0.1; 0.01; 0.01]);
elseif all(ismember(filename,'DataMapping.mat')) || all(ismember(filename,'DataSquare.mat')) || all(ismember(filename,'DataStraightLine.mat'))
    noise1 = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
    noise2 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 1; 0.01; 0.01; 0.01]);
elseif all(ismember(filename,'DataSlowCircle.mat')) || all(ismember(filename,'DataMountain.mat'))
    noise1 = noiseModel.Diagonal.Sigmas([0.001; 0.001; 0.001]);
    noise2 = noiseModel.Diagonal.Sigmas([1; 10; 10; 0.1; 1; 0.1]);
end
noise3 = noiseModel.Diagonal.Sigmas([3; 3]);

initialEstimate = Values;
for i=1:size(LandMarksComputed,1)
    for j=1:4
        % Add prior for Tags
        graph.add(PriorFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+j), Point3(LandMarksComputed(i,2*j),LandMarksComputed(i,2*j+1),0), noise1));
        % Create initial estimate for Tags
        initialEstimate.insert(symbol('l',LandMarksComputed(i,1)*10+j), Point3(LandMarksComputed(i,2*j),LandMarksComputed(i,2*j+1),0));
    end
    % Add constraint
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+1),symbol('l',LandMarksComputed(i,1)*10+2),Point3(TagSize,0,0),noise1));
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+2),symbol('l',LandMarksComputed(i,1)*10+3),Point3(0,TagSize,0),noise1));
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+3),symbol('l',LandMarksComputed(i,1)*10+4),Point3(-TagSize,0,0),noise1));
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+4),symbol('l',LandMarksComputed(i,1)*10+1),Point3(0,-TagSize,0),noise1));
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+1),symbol('l',LandMarksComputed(i,1)*10+3),Point3(TagSize,TagSize,0),noise1));
    graph.add(BetweenFactorPoint3(symbol('l',LandMarksComputed(i,1)*10+2),symbol('l',LandMarksComputed(i,1)*10+4),Point3(-TagSize,TagSize,0),noise1));
end

% Add prior for first camera pose
intrinsic = Cal3_S2(K(1,1),K(2,2),K(1,2),K(1,3),K(2,3));
imagePointsInitialized = [];
worldPointsInitialized = [];
for j=1:size(DetAll{1},1)
    % Add GenericProjectionFactor
    for k=1:4
        graph.add(GenericProjectionFactorCal3_S2(Point2(DetAll{1}(j,2*k),DetAll{1}(j,2*k+1)),noise3,symbol('x',1),symbol('l',DetAll{1}(j,1)*10+k),intrinsic));
    end
    imagePoints = reshape(DetAll{1}(j,2:9),[2,4]);
    imagePointsInitialized = [imagePointsInitialized,imagePoints];
    index = find(LandMarksComputed(:,1)==DetAll{1}(j,1),1);
    worldPoints = reshape(LandMarksComputed(index,2:9),[2,4]);
    worldPointsInitialized = [worldPointsInitialized,worldPoints];
end
H = homography(worldPointsInitialized,imagePointsInitialized);
B = K\H;
R = [B(:,1)/norm(B(:,1)) B(:,2)/norm(B(:,2)) cross(B(:,1),B(:,2))/norm(cross(B(:,1),B(:,2)))];
T = B(:,3);
graph.add(PriorFactorPose3(symbol('x',1), Pose3(Rot3(R),Point3(T)), noise2));
% Create initial estimate for first camera pose
initialEstimate.insert(symbol('x',1), Pose3(Rot3(R),Point3(T)));

% Initial Optimization
batchOptimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
initialEstimate = batchOptimizer.optimize();

% Initialize iSAM
isam = ISAM2;

% update iSAM
isam.update(graph, initialEstimate);
result = isam.calculateEstimate();

% Loop
for i=2:length(DetAll)
    % update landmark estimates
    graph = NonlinearFactorGraph;
    initialEstimate = Values;
    if ~isempty(DetAll{i})
        imagePointsInitialized = [];
        worldPointsInitialized = [];
        for j=1:size(DetAll{i},1)
            % Add GenericProjectionFactor
            for k=1:4
                graph.add(GenericProjectionFactorCal3_S2(Point2(DetAll{i}(j,2*k),DetAll{i}(j,2*k+1)),noise3,symbol('x',i),symbol('l',DetAll{i}(j,1)*10+k),intrinsic));
            end
            % Create initial estimate
            imagePoints = reshape(DetAll{i}(j,2:9),[2,4]);
            imagePointsInitialized = [imagePointsInitialized,imagePoints];
            index = find(LandMarksComputed(:,1)==DetAll{i}(j,1),1);
            worldPoints = reshape(LandMarksComputed(index,2:9),[2,4]);
            worldPointsInitialized = [worldPointsInitialized,worldPoints];
        end
        H = homography(worldPointsInitialized,imagePointsInitialized);
        B = K\H;
        R = [B(:,1)/norm(B(:,1)) B(:,2)/norm(B(:,2)) cross(B(:,1),B(:,2))/norm(cross(B(:,1),B(:,2)))];
        T = B(:,3);
    end
    
    % Add odometry
    initialEstimate.insert(symbol('x',i), Pose3(Rot3(R),Point3(T)));
    graph.add(BetweenFactorPose3(symbol('x',i-1), symbol('x',i), Pose3(), noise2));
    try
        % update iSAM
        isam.update(graph, initialEstimate);
        result = isam.calculateEstimate();
    catch
        continue;
    end
end

AllPosesComputed = zeros(length(DetAll),7);
for i=1:length(DetAll)
    try
        T = matrix(result.at(symbol('x',i)));
        AllPosesComputed(i,:) = [T(1:3,4)',RotToQuat(T(1:3,1:3))'];
    catch
        AllPosesComputed(i,:) = [T(1:3,4)',RotToQuat(T(1:3,1:3))'];
    end
end

end