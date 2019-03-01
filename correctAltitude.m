function Z = correctAltitude(AllPosesComputed, LandMarksComputed, TagSize, K)
% This function uses the LandMarksComputed to determine the rotation and
% translation between this and real world coordinates and then adjust
% AllPosesComputed accordingly.

estPoints = reshape(LandMarksComputed(LandMarksComputed(:,1)==10,2:9),[2,4]);
realPoints = [0 0;  TagSize 0; TagSize TagSize;0 TagSize]';
H = homography(estPoints,realPoints);
B = K\H;
R = [B(:,1)/norm(B(:,1)) B(:,2)/norm(B(:,2)) cross(B(:,1),B(:,2))/norm(cross(B(:,1),B(:,2)))];
T = [R, B(:,3); 0 0 0 1];
pos = T\[AllPosesComputed(:,1:3)'; ones(1,size(AllPosesComputed,1))];
Z = pos(3,:)';
            
end