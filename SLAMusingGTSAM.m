function [LandMarksComputed, AllPosesComputed, covariance] = SLAMusingGTSAM(DetAll, K, TagSize, filename)
% For Input and Output specifications refer to the project pdf

% import
import gtsam.*

% Create graph container and add factors to it
graph = NonlinearFactorGraph;

% define noise parameters
noise1 = noiseModel.Diagonal.Sigmas([0.001; 0.001; 0.001]);
noise2 = noiseModel.Diagonal.Sigmas([3; 3]);
if all(ismember(filename,'DataFastCircle.mat'))
    noise3 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.1; 0.001; 1; 0.01]);
elseif all(ismember(filename,'DataMapping.mat')) || all(ismember(filename,'DataSquare.mat')) || all(ismember(filename,'DataStraightLine.mat'))
    noise3 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.05; 0.05; 0.01; 0.05]);
elseif all(ismember(filename,'DataSlowCircle.mat'))
    noise3 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.01; 0.01; 0.1; 0.01]);
elseif  all(ismember(filename,'DataMountain.mat'))
    noise3 = noiseModel.Diagonal.Sigmas([10; 0.01; 10; 0.01; 10; 0.1]);
end

% Add prior for Tag10
graph.add(PriorFactorPoint3(symbol('l',101), Point3(0,0,0), noise1));
graph.add(PriorFactorPoint3(symbol('l',102), Point3(TagSize,0,0), noise1));
graph.add(PriorFactorPoint3(symbol('l',103), Point3(TagSize,TagSize,0), noise1));
graph.add(PriorFactorPoint3(symbol('l',104), Point3(0,TagSize,0), noise1));

intrinsic = Cal3_S2(K(1,1),K(2,2),K(1,2),K(1,3),K(2,3));
% Create initial estimate
initialEstimate = Values;
tagsInitialized = [10 0 0 TagSize 0 TagSize TagSize 0 TagSize];
for i=1:4
    initialEstimate.insert(symbol('l',100+i), Point3(tagsInitialized(1,2*i),tagsInitialized(1,2*i+1),0));
end
uninitializedFrames = [];
for i=1:length(DetAll)
    imagePoints2Initialize = [];
    imagePointsInitialized = [];
    worldPointsInitialized = [];
    if ~isempty(DetAll{i})
        for j=1:size(DetAll{i},1)
            if isempty(find(tagsInitialized(:,1) == DetAll{i}(j,1),1))
                % Add constraint
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+2),Point3(TagSize,0,0),noise1));
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+3),Point3(0,TagSize,0),noise1));
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+3),symbol('l',DetAll{i}(j,1)*10+4),Point3(-TagSize,0,0),noise1));
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+4),symbol('l',DetAll{i}(j,1)*10+1),Point3(0,-TagSize,0),noise1));
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+3),Point3(TagSize,TagSize,0),noise1));
                graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+4),Point3(-TagSize,TagSize,0),noise1));
                % Initial Estimate
                imagePoints = [symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+3),symbol('l',DetAll{i}(j,1)*10+4);...
                               reshape(DetAll{i}(j,2:9),[2,4]); 1 1 1 1];
                imagePoints2Initialize = [imagePoints2Initialize,imagePoints];
            else
                imagePoints = reshape(DetAll{i}(j,2:9),[2,4]);
                imagePointsInitialized = [imagePointsInitialized,imagePoints];
                index = find(tagsInitialized(:,1) == DetAll{i}(j,1),1);
                worldPoints = reshape(tagsInitialized(index,2:9),[2,4]);
                worldPointsInitialized = [worldPointsInitialized,worldPoints];
            end
            % Add GenericProjectionFactor
            for k=1:4
            graph.add(GenericProjectionFactorCal3_S2(Point2(DetAll{i}(j,2*k),DetAll{i}(j,2*k+1)),noise2,symbol('x',i),symbol('l',DetAll{i}(j,1)*10+k),intrinsic));
            end
        end
    end
    % Add odometry
    if i~=1
        graph.add(BetweenFactorPose3(symbol('x',i-1), symbol('x',i), Pose3(), noise3));
    end
    % Initial Estimate
    if ~isempty(imagePointsInitialized)
        if ~isempty(imagePoints2Initialize)
            H = homography(worldPointsInitialized,imagePointsInitialized,'ransac');
            worldPoints2Initialize = H\double(imagePoints2Initialize(2:4,:));
            worldPoints2Initialize = [worldPoints2Initialize(1,:)./worldPoints2Initialize(3,:); worldPoints2Initialize(2,:)./worldPoints2Initialize(3,:)];
            for k=1:length(imagePoints2Initialize)
                initialEstimate.insert(imagePoints2Initialize(1,k), Point3([worldPoints2Initialize(:,k);0]));
                if mod(k,4)==1
                    tagsInitialized = [tagsInitialized;double(floor((imagePoints2Initialize(1,k)-symbol('l',0))/10)) reshape(worldPoints2Initialize(:,k:k+3),[1,8])];
                end
            end
            imagePointsInitialized = [imagePointsInitialized,double(imagePoints2Initialize(2:3,:))];
            worldPointsInitialized = [worldPointsInitialized,worldPoints2Initialize];
        end
        H = homography(worldPointsInitialized,imagePointsInitialized,'ransac');
        B = K\H;
        R = [B(:,1)/norm(B(:,1)) B(:,2)/norm(B(:,2)) cross(B(:,1),B(:,2))/norm(cross(B(:,1),B(:,2)))];
        T = B(:,3);
        % Add prior for first camera pose
        if i==1
            graph.add(PriorFactorPose3(symbol('x',i), Pose3(Rot3(R),Point3(T)), noise3));
        end
        initialEstimate.insert(symbol('x',i), Pose3(Rot3(R),Point3(T)));
    else
        uninitializedFrames = [uninitializedFrames;i];
    end
end
if ~isempty(uninitializedFrames)
    for z=1:length(uninitializedFrames)
        i = uninitializedFrames(z);
        imagePoints2Initialize = [];
        imagePointsInitialized = [];
        worldPointsInitialized = [];
        if ~isempty(DetAll{i})
            for j=1:size(DetAll{i},1)
                if isempty(find(tagsInitialized(:,1) == DetAll{i}(j,1),1))
                    % Add constraint
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+2),Point3(TagSize,0,0),noise1));
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+3),Point3(0,TagSize,0),noise1));
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+3),symbol('l',DetAll{i}(j,1)*10+4),Point3(-TagSize,0,0),noise1));
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+4),symbol('l',DetAll{i}(j,1)*10+1),Point3(0,-TagSize,0),noise1));
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+3),Point3(TagSize,TagSize,0),noise1));
                    graph.add(BetweenFactorPoint3(symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+4),Point3(-TagSize,TagSize,0),noise1));
                    % Initial Estimate
                    imagePoints = [symbol('l',DetAll{i}(j,1)*10+1),symbol('l',DetAll{i}(j,1)*10+2),symbol('l',DetAll{i}(j,1)*10+3),symbol('l',DetAll{i}(j,1)*10+4);...
                                   reshape(DetAll{i}(j,2:9),[2,4]); 1 1 1 1];
                    imagePoints2Initialize = [imagePoints2Initialize,imagePoints];
                else
                    imagePoints = reshape(DetAll{i}(j,2:9),[2,4]);
                    imagePointsInitialized = [imagePointsInitialized,imagePoints];
                    index = find(tagsInitialized(:,1) == DetAll{i}(j,1),1);
                    worldPoints = reshape(tagsInitialized(index,2:9),[2,4]);
                    worldPointsInitialized = [worldPointsInitialized,worldPoints];
                end
            end
        end
        if ~isempty(imagePointsInitialized)
            if ~isempty(imagePoints2Initialize)
                H = homography(worldPointsInitialized,imagePointsInitialized,'ransac');
                worldPoints2Initialize = H\double(imagePoints2Initialize(2:4,:));
                worldPoints2Initialize = [worldPoints2Initialize(1,:)./worldPoints2Initialize(3,:); worldPoints2Initialize(2,:)./worldPoints2Initialize(3,:)];
                for k=1:length(imagePoints2Initialize)
                    initialEstimate.insert(imagePoints2Initialize(1,k), Point3([worldPoints2Initialize(:,k);0]));
                    if mod(k,4)==1
                        tagsInitialized = [tagsInitialized;double(floor((imagePoints2Initialize(1,k)-symbol('l',0))/10)) reshape(worldPoints2Initialize(:,k:k+3),[1,8])];
                    end
                end
                imagePointsInitialized = [imagePointsInitialized,double(imagePoints2Initialize(2:3,:))];
                worldPointsInitialized = [worldPointsInitialized,worldPoints2Initialize];
            end
            H = homography(worldPointsInitialized,imagePointsInitialized,'ransac');
            B = K\H;
            R = [B(:,1)/norm(B(:,1)) B(:,2)/norm(B(:,2)) cross(B(:,1),B(:,2))/norm(cross(B(:,1),B(:,2)))];
            T = B(:,3);
            % Add prior for first camera pose
            if i==1
                graph.add(PriorFactorPose3(symbol('x',i), Pose3(Rot3(R),Point3(T)), noise3));
            end
            initialEstimate.insert(symbol('x',i), Pose3(Rot3(R),Point3(T)));
        else
            uninitializedFrames = [uninitializedFrames;i];
        end
    end
    if z<length(uninitializedFrames)
        for z=z+1:length(uninitializedFrames)
            i = uninitializedFrames(z);
            if i==1
                k = k+i;
                while ~isempty(find(uninitializedFrames==k,1))
                    k = k+1;
                end
                graph.add(PriorFactorPose3(symbol('x',i), initialEstimate.at(symbol('x',k)), noise3));
                initialEstimate.insert(symbol('x',i), initialEstimate.at(symbol('x',k)));
            else
                initialEstimate.insert(symbol('x',i), initialEstimate.at(symbol('x',i-1)));
            end
        end
    end
end
[~,I] = sort(tagsInitialized(:,1));
tagsInitialized = tagsInitialized(I,:);

% Optimize
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
for i=1:5
    optimizer.iterate;
end
result = optimizer.optimize();

AllPosesComputed = zeros(length(DetAll),7);
LandMarksComputed = zeros(size(tagsInitialized,1),9);
for i=1:length(DetAll)
    T = matrix(result.at(symbol('x',i)));
    AllPosesComputed(i,:) = [T(1:3,4)',RotToQuat(T(1:3,1:3))'];
end
for i=1:size(tagsInitialized,1)
    LandMarksComputed(i,1) = tagsInitialized(i,1);
    for j=1:4
        LandMarksComputed(i,2*j) = result.at(symbol('l',tagsInitialized(i,1)*10+j)).x;
        LandMarksComputed(i,2*j+1) = result.at(symbol('l',tagsInitialized(i,1)*10+j)).y;
    end
end

marginals = Marginals(graph, result);
covariance = marginals.marginalCovariance(symbol('x',1));

end