function Vel = velocityEstimator( LeftImgs, dT, K, Z )
% This function takes the images, time difference, camera calibration
% matrix and real depth to the center of the image 

    Vel = zeros(1,6);
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
        count = count + 1;
        newOldPoints = oldPoints(isFound, :);
        % Optical Flow Estimation
        u = (newPoints(:,1)-newOldPoints(:,1))*30/(count*dT);
        v = (newPoints(:,2)-newOldPoints(:,2))*30/(count*dT);
        % Velocity Estimation using ransac()
        indexUsed = zeros(10000,3);
        leastError = Inf;
        staticErrorCount = 0;
        for j=1:10000
            % find unique set of 3 u,v
            index = randperm(length(u),3);
            index = sort(index);
            if ~isempty(indexUsed)
                match_1 = indexUsed(:,1) == index(1);
                if any(match_1)
                    match_2 = indexUsed(match_1,2) == index(2);
                    if any(match_2)
                        match_1 = find(match_1);
                        match_3 = indexUsed(match_1(match_2),3) == index(3);
                        if any(match_3)
                            continue;
                        end
                    end
                end
            end
            indexUsed(j,:) = index;
            % estimate velocity
            ui = u(index); vi = v(index);
            x = newPoints(index,1) - K(1,3); y = newPoints(index,2) - K(2,3);
            P = (1 - K(1,1)*x - K(2,2)*y)/Z(i);
            Ua = x(1)*y(1)/P(1); Ub = -(1 + x(1)^2)/P(1); Uc = y(1)/P(1); Uk = -ui(1)/P(1);
            Va = (1 + y(1)^2)/P(1); Vc = -x(1)/P(1); Vk = -vi(1)/P(1);
            Wa = (P(2)*Ua - x(2)*y(2))/(P(2)*(x(2) - x(1))); Wb = (1 + x(2)^2 + P(2)*Ub)/(P(2)*(x(2) - x(1)));
            Wc = (P(2)*Uc - y(2))/(P(2)*(x(2) - x(1))); Wk = (P(2)*Uk + ui(2))/(P(2)*(x(2) - x(1)));
            ab = (x(2)*y(2) - P(2)*(Ua + Wb*(y(2) - y(1))))/(1 + y(2)^2 - P(2)*(Va - Wa*(y(2) - y(1))));
            ac = (x(2) + P(2)*(Vc - Wc*(y(2) - y(1))))/(1 + y(2)^2 - P(2)*(Va - Wa*(y(2) - y(1))));
            ak = (vi(2) + P(2)*(Vk - Wk*(y(2) - y(1))))/(1 + y(2)^2 - P(2)*(Va - Wa*(y(2) - y(1))));
            bc = (y(3) - P(3)*(Uc - Wc*(x(3) - x(1))) + ac*(x(3)*y(3) - P(3)*(Ua - Wa*(x(3) - x(1)))))/(1 + x(3)^2 + P(3)*(Ub - Wb*(x(3) - x(1))) - ab*(x(3)*y(3) - P(3)*(Ua - Wa*(x(3) - x(1)))));
            bk = (P(3)*(Wk*(x(3) - x(1)) - Uk) + ak*(x(3)*y(3) - P(3)*(Ua - Wa*(x(3) - x(1)))) - ui(3))/(1 + x(3)^2 + P(3)*(Ub - Wb*(x(3) - x(1))) - ab*(x(3)*y(3) - P(3)*(Ua - Wa*(x(3) - x(1)))));
            cN = P(3)*(Wk*(y(3) - y(1)) - Vk) + ak*(1 + y(3)^2 - P(3)*(Va - Wa*(y(3) - y(1)))) - vi(3) - bk*(x(3)*y(3) - P(3)*(Ua + Wb*(y(3) - y(1))) - ab*(1 + y(3)^2 - P(3)*(Va - Wa*(y(3) - y(1)))));
            cD = x(3) + P(3)*(Vc - Wc*(y(3) - y(1))) - ac*(1 + y(3)^2 - P(3)*(Va - Wa*(y(3) - y(1)))) + bc*(x(3)*y(3) - P(3)*(Ua + Wb*(y(3) - y(1))) - ab*(1 + y(3)^2 - P(3)*(Va - Wa*(y(3) - y(1)))));
            c = cN/cD;
            b = bc*c + bk;
            a = ab*b + ac*c + ak;
            W = Wa*a + Wb*b + Wc*c + Wk;
            V = y(1)*W + Va*a - Ua*b + Vc*c + Vk;
            U = x(1)*W + Ua*a + Ub*b + Uc*c + Uk;
            % check error
            x = newPoints(:,1) - K(1,3); y = newPoints(:,2) - K(2,3);
            P = (1 - K(1,1)*x - K(2,2)*y)/Z(i);
            u_temp = W*P.*(x - U/W) + a*x.*y - b*(1 + x.^2) + c*y;
            v_temp = W*P.*(y - V/W) + a*(1 + y.^2) - b*x.*y - c*x;
            error = sqrt((mean((u-u_temp).^2) + mean((v-v_temp).^2))/2);
            % store Velocity associated with least error
            if error < leastError
                leastError = error;
                Vel(i,:) = [double(U) double(V) double(W) double(a) double(b) double(c)];
                staticErrorCount = 0;
            elseif ((j > 2000)&&(leastError < 25)) || (j > 5000)
                staticErrorCount = staticErrorCount + 1;
                if staticErrorCount == 1000
                    break;
                end
            end
        end
    end
end