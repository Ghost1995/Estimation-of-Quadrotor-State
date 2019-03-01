function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, ~, ~, ~, ~, ~, LandMarksComputed)
% For Input and Output specifications refer to the project pdf

import gtsam.*

% Create graph container and add factors to it
graph = NonlinearFactorGraph;

noise1 = noiseModel.Diagonal.Sigmas([0.001; 0.001; 0.001]);
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
noise2 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.01; 0.01; 0.01; 0.01]);
noise3 = noiseModel.Diagonal.Sigmas([3; 3]);
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
        if i~=1
            if abs(T1(1,4)-T(1,4))>10 || abs(T1(2,4)-T(2,4))>10 || abs(T1(2,4)-T(2,4))>10
                T = T1;
            else
                T1 = T;
            end
        end
        T1 = T;
        AllPosesComputed(i,:) = [T(1:3,4)',RotToQuat(T(1:3,1:3))'];
    catch
        AllPosesComputed(i,:) = [T(1:3,4)',RotToQuat(T(1:3,1:3))'];
    end
end

end