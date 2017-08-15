close all;

% a5 = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
[lenA,~] = size(a);
[lenO,~] = size(o);
lenG = min(lenA,lenO);
vectorG = zeros(lenG,3);
o1 = o(1:lenG,:);
a1 = a(1:lenG,:);
to1 = to(1:lenG);
t1 = t(1:lenG);
quat = angle2quat(deg2rad(o1(:,1)),deg2rad(o1(:,2)),deg2rad(o1(:,3)));

vectorG(:,1) = 2 .* ((quat(:,2) .* quat(:,4)) - (quat(:,1) .* quat(:,3)));
vectorG(:,2) = 2 * ((quat(:,1) .* quat(:,2)) + (quat(:,3) .* quat(:,4)));
vectorG(:,3) = quat(:,1).^2 - quat(:,2).^2 - quat(:,3).^2 + quat(:,4).^2;

% a2 = vectorG - a1;
% for : = 1:lenG
%     vectorG(:,1) = 2 * ((quat(:,2) * quat(:,4)) - (quat(:,1) * quat(:,3)));
%     vectorG(:,2) = 2 * ((quat(:,1) * quat(:,2)) + (quat(:,3) * quat(:,4)));
%     vectorG(:,3) = quat(:,1)^2 - quat(:,2)^2 - quat(:,3)^2 + quat(:,4)^2;