[lenPos,~] = size(odomfiltData);
odomDistance = zeros(1,lenPos);

for i = 1:lenPos
pos1 = [odomfiltData{1}.Pose.Pose.Position.X,...
    odomfiltData{1}.Pose.Pose.Position.Y,odomfiltData{1}.Pose.Pose.Position.Z];

pos2 = [odomfiltData{i}.Pose.Pose.Position.X,...
    odomfiltData{i}.Pose.Pose.Position.Y,odomfiltData{i}.Pose.Pose.Position.Z];

% pos2 = [odomfiltData{lenPos}.Pose.Pose.Position.X,...
%     odomfiltData{lenPos}.Pose.Pose.Position.Y,odomfiltData{lenPos}.Pose.Pose.Position.Z];
odomDistance(i) = norm([pos1;pos2]);

poss(i,:) = [odomfiltData{i}.Pose.Pose.Position.X,...
    odomfiltData{i}.Pose.Pose.Position.Y,odomfiltData{i}.Pose.Pose.Position.Z];
end

plot(odomDistance)
odomDistance(i)
figure,
plot(poss)