function mu = extendedKalmanFilter( Pose, Velocity, unsyncedIMU, TLeftImgs, covariance )

% sync IMU data with camera data
syncedIMU = zeros(length(TLeftImgs),6);
oldIndex = 0; unusedIndex = [];
for i=1:length(TLeftImgs)
    index = find(unsyncedIMU(:,11) >= TLeftImgs(i),1);
    if ~isempty(index)
        if (index ~= 1)&&(abs(unsyncedIMU(index,11)-TLeftImgs(i)) > abs(unsyncedIMU(index-1,11)-TLeftImgs(i)))
            if (index-1) == oldIndex
                unusedIndex = [unusedIndex;i];
                continue;
            end
            oldIndex = index-1;
            syncedIMU(i,:) = unsyncedIMU(index-1,5:10);
            TLeftImgs(i) = unsyncedIMU(index-1,11);
        else
            if index == oldIndex
                unusedIndex = [unusedIndex;i];
                continue;
            end
            oldIndex = index;
            syncedIMU(i,:) = unsyncedIMU(index,5:10);
            TLeftImgs(i) = unsyncedIMU(index,11);
        end
    elseif oldIndex ~= size(unsyncedIMU,1)
        TLeftImgs(i) = unsyncedIMU(end,11);
        unusedIndex = [unusedIndex;(i:length(TLeftImgs))'];
        break;
    end
end
syncedIMU(unusedIndex,:) = [];
TLeftImgs(unusedIndex,:) = [];
Pose(unusedIndex,:) = [];
Velocity(unusedIndex,:) = [];
dT = zeros(length(TLeftImgs),1);
dT(2:end) = TLeftImgs(2:end) - TLeftImgs(1:end-1);

mu_hat = zeros(length(TLeftImgs),15);
mu = zeros(length(TLeftImgs),15);
% Initialize mu and sigma
mu(1,:) = [Pose(1,1:3), RotToRPY_ZXY(QuatToRot(Pose(1,4:7))), Velocity(1,:), 0.012208142813, -0.004909187064, 0.004946068042, -0.089717962548, -0.175323465945, 0.206834236515];
sigma = [covariance, zeros(9,6); zeros(6,15)];
% Initialize Q & R
Q = 1.5e-2;
R = 1.5e-5;
% Initialize C & W
C = [eye(9),zeros(9,6)];
W = eye(9);

for i=2:length(TLeftImgs)
    % compute jacobian
    Xdot = getXdot(mu(i-1,:),syncedIMU(i,:),0);
    A = jacobianMatrix_A(mu(i-1,:),syncedIMU(i,:),0);
    U = jacobianMatrix_U(mu(i-1,:));

    % Prediction Step
    mu_hat(i,:) = mu(i-1,:) + (Xdot*dT(i))';
    sigma_hat = (eye(15) + A*dT(i))*sigma*(eye(15) + A*dT(i))' + (U*dT(i))*Q*(U*dT(i))';

    % measurement step
    K = (sigma_hat*C')/(C*sigma_hat*C' + W*R*W');
    sigma = (eye(15) - K*C)*sigma_hat;
    X = [Pose(i-1,1:3)'; RotToRPY_ZXY(QuatToRot(Pose(i-1,4:7)))'; Velocity(i-1,:)';  mu(1,10:15)'];
    n = normrnd(0,Q,[12 1]);
    X_hat = X + getXdot(X,syncedIMU(i,:),n)*dT(i);
    v = normrnd(0,R,[9 1]);
    mu(i,:) = mu_hat(i,:) + (K*(C*(X_hat - mu_hat(i,:)') + W*v))';
end

for i=1:length(unusedIndex)
    if unusedIndex(i) <= size(mu,1)
        mu(unusedIndex(i)+1:end+1,:) = mu(unusedIndex(i):end,:);
        mu(unusedIndex(i),:) = mean([mu(unusedIndex(i)-1,:);mu(unusedIndex(i)+1,:)]);
    else
        mu(unusedIndex(i),:) = 2*mu(unusedIndex(i)-1,:) - mu(unusedIndex(i)-2,:);
    end
end
mu = mu(:,1:9);
end