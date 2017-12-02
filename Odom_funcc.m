[lenPos,~] = size(odomfiltData);
odomDistance_overall = zeros(1,lenPos);
odomDistance_overall_x = zeros(1,lenPos)
odomDistance_overall_y = zeros(1,lenPos)

for i = 1:lenPos
pos1 = [odomfiltData{1}.Pose.Pose.Position.X,...
    odomfiltData{1}.Pose.Pose.Position.Y,odomfiltData{1}.Pose.Pose.Position.Z];

pos2 = [odomfiltData{i}.Pose.Pose.Position.X,...
    odomfiltData{i}.Pose.Pose.Position.Y,odomfiltData{i}.Pose.Pose.Position.Z];

% pos2 = [odomfiltData{lenPos}.Pose.Pose.Position.X,...
%     odomfiltData{lenPos}.Pose.Pose.Position.Y,odomfiltData{lenPos}.Pose.Pose.Position.Z];

odomDistance_overall(i) = norm([pos1;pos2]);

poss(i,:) = [odomfiltData{i}.Pose.Pose.Position.X,...
    odomfiltData{i}.Pose.Pose.Position.Y,odomfiltData{i}.Pose.Pose.Position.Z];
end

% figure,
% plot(odomDistance_overall);
% odomDistance_overall(i);
% figure,
% plot(poss);