close all;

weightGyro = 5;
aRaw = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
avRaw0 = [Untitled.Gyroscope_x,Untitled.Gyroscope_y,Untitled.Gyroscope_z];

avRaw = avRaw0;
for i = 1:3
    avRaw(:,i) = wden(avRaw0(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    avRaw(:,i) = avRaw(:,i) - mean(avRaw(1:300,i));
end

[lenA,~] = size(avRaw);
[lenAv,~] = size(aRaw);
len = min(lenA,lenAv);

t = Untitled.Time(1:len);
aEst = zeros(len,3,'double');
anglesPrev = [rad2deg(atan2(aRaw(1,1), aRaw(1,3))),...
    rad2deg(atan2(aRaw(1,2), aRaw(1,3)))];
aEst(1,:) = aRaw(1,:);

for i = 2:len
    [aEst(i,:),anglesPrev] = findEstimate(avRaw(i,:), aEst(i-1,:), avRaw(i-1,:), anglesPrev);
end
aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);
plot(t,aAdjusted);
figure,plot(t,aEst);
figure,plot(t,aRaw);
figure,plot(t,avRaw);

anglesSph = zeros(len,3,'double');
[anglesSph(:,1),anglesSph(:,2),anglesSph(:,3)] = cart2sph(aEst(:,1),aEst(:,2),aEst(:,3));
anglesSph(:,1:2) = rad2deg(anglesSph(:,1:2));
figure,plot(t,anglesSph(:,1:2));

% weightGyro = 1;
% aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);
% figure,plot(t,aAdjusted);
% weightGyro = 10;
% aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);
% figure,plot(t,aAdjusted);
% weightGyro = 20;
% aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);
% figure,plot(t,aAdjusted);

function [aEst,angles] = findEstimate(avCurrentRaw, aPreviousEst, avPreviousEst, anglesPrev)
    timePeriod = 0.01;
    aEst = zeros(1,3,'double');    
    avXZAvg = mean([avCurrentRaw(2),avPreviousEst(2)]);
    angleXZcurr = anglesPrev(1) + avXZAvg * timePeriod;
    avYZAvg = mean([avCurrentRaw(1),avPreviousEst(1)]);
    angleYZcurr = anglesPrev(2) + avYZAvg * timePeriod;
    angles = [angleXZcurr,angleYZcurr];
    
    aEst(1) =  1  / sqrt(1  +   cotd(angleXZcurr)^2 * secd(angleYZcurr)^2 );
    aEst(2) =  1  / sqrt(1  +   cotd(angleYZcurr)^2 * secd(angleXZcurr)^2 );
    aEst(3) =  sign(aPreviousEst(3)) * sqrt(1 - aEst(2)^2 - aEst(3)^2);
end