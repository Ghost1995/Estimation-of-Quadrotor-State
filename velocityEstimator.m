function Vel = velocityEstimator( LeftImgs, dT, K, Z )
% This function takes the images, time difference, camera calibration
% matrix and real depth to the center of the image 

    % applying a low pass filter
    filter = mean(dT(2:end));
    dTusedIndex = dT <= filter;
    while sum(dTusedIndex) < 0.8*length(dT)
        filter = filter + mean(dT(2:end));
        dTusedIndex = dT <= filter;
    end
    dTused = dT(dTusedIndex);
    LeftImgs = LeftImgs(:,:,dTusedIndex);
    Vel = zeros(length(dTused),6);
    I_old = LeftImgs(:,:,1);
    % Corner Detection
    oldPoints = detectMinEigenFeatures(I_old,'MinQuality', 0.1);
    oldPoints = oldPoints.Location;
    % KLT Tracker
    pointTracker = vision.PointTracker;
    initialize(pointTracker, oldPoints, I_old);
    count = 0;
    for i=2:size(LeftImgs,3)
        I_new = LeftImgs(:,:,i);
        [newPoints, isFound] = step(pointTracker, I_new);
        newPoints = newPoints(isFound, :);
        if length(newPoints) < 0.8*length(oldPoints)
            count = 0;
            I_old = LeftImgs(:,:,i-1);
            % Corner Detection
            oldPoints = detectMinEigenFeatures(I_old,'MinQuality', 0.1);
            oldPoints = oldPoints.Location;
            % KLT Tracker
            setPoints(pointTracker, oldPoints);
            [newPoints, isFound] = step(pointTracker, I_new);
            newPoints = newPoints(isFound, :);
        end
        newOldPoints = oldPoints(isFound, :);
        % Optical Flow Estimation
        u = (newPoints(:,1)-newOldPoints(:,1))/sum(dT(i-count:i));
        v = (newPoints(:,2)-newOldPoints(:,2))/sum(dT(i-count:i));
        count = count + 1;
        % Velocity Estimation using ransac()
        indexUsed = zeros(500,3);
        leastError = Inf;
        for j=1:500
            indexUsed(j,:) = randperm(length(u),3);
            % estimate velocity
            ui = u(indexUsed(j,:)); vi = v(indexUsed(j,:));
            x = newPoints(indexUsed(j,:),1) - K(1,3); y = newPoints(indexUsed(j,:),2) - K(2,3);
            P = (1 - K(1,1)*x - K(2,2)*y)/Z(i);
            T = [-P zeros(3,1) P.*x x.*y -(1+x.^2) y; zeros(3,1) -P P.*y (1+y.^2) -x.*y -x];
            vel = T\[ui;vi];
            % check error
            x = newPoints(:,1) - K(1,3); y = newPoints(:,2) - K(2,3);
            P = (1 - K(1,1)*x - K(2,2)*y)/Z(i);
            T = [-P zeros(length(P),1) P.*x x.*y -(1+x.^2) y; zeros(length(P),1) -P P.*y (1+y.^2) -x.*y -x];
            flowVel = T*vel;
            error = (sqrt(mean((u-flowVel(1:length(flowVel)/2)).^2)) + sqrt(mean((v-flowVel((length(flowVel)/2)+1:end)).^2)))/2;
            % store Velocity associated with least error
            if error < leastError
                leastError = error;
                Inliers = (u-flowVel(1:length(flowVel)/2) <= error)&(v-flowVel((length(flowVel)/2)+1:end) <= error);
            end
        end
        x = newPoints(Inliers,1) - K(1,3); y = newPoints(Inliers,2) - K(2,3);
        P = (1 - K(1,1)*x - K(2,2)*y)/Z(i);
        T = [-P zeros(length(P),1) P.*x x.*y -(1+x.^2) y; zeros(length(P),1) -P P.*y (1+y.^2) -x.*y -x];
        Vel(i,:) = (T\[u(Inliers);v(Inliers)])';
    end
    
    for i=1:length(dTusedIndex)
        if ~dTusedIndex(i)
            if i <= size(Vel,1)
                Vel(i+1:end+1,:) = Vel(i:end,:);
                Vel(i,:) = mean([Vel(i-1,:);Vel(i+1,:)]);
            else
                Vel(i,:) = 2*Vel(i-1,:) - Vel(i-2,:);
            end
        end
    end
end