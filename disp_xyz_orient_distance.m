close all;

% importRaspPi;
weightGyro = 20;
aRaw0 = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
avRaw0 = [Untitled.Gyroscope_x,Untitled.Gyroscope_y,Untitled.Gyroscope_z];
% aRaw0 = a;
% avRaw0 = av;
[lenA,~] = size(avRaw0);
[lenAv,~] = size(aRaw0);
len = min(lenA,lenAv);
t = Untitled.Time(1:len);

avRaw = avRaw0;
aRaw1 = aRaw0;
aRaw = aRaw0;

subplot(4,1,1);
plot(t,aRaw0);
for i = 1:3
    avRaw(:,i) = wden(avRaw0(:,i),'modwtsqtwolog','s','mln',8,'sym4');
%     aRaw1(:,i) = wden(aRaw0(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    
    subplot(4,1,2);
    plot(t,aRaw1);
    order = 6; % 6th Order Filter
    [b1,a1] = butter(order, 0.01, 'low');
    aRaw(:,i) = filtfilt(b1 ,a1 , aRaw1(:,i));
    subplot(4,1,3);
    plot(t,aRaw);
    
    aRaw1(:,i) = medfilt1(aRaw1(:,i),100);
    aRaw1(1,i) = aRaw0(1,i);
    subplot(4,1,4);
    plot(t,aRaw1);
    
    avRaw(:,i) = avRaw(:,i) - mean(avRaw(1:300,i));
    avRaw(:,i) = medfilt1(avRaw(:,i),100);
end
figure;

gSum = zeros(1,'double');
for i = 1:300
    gSum = gSum + norm(aRaw0(i,:));
end
gMean = gSum / 300

aEst = zeros(len,3,'double');
anglesPrev = [rad2deg(atan2(aRaw(1,1), aRaw(1,3))),...
    rad2deg(atan2(aRaw(1,2), aRaw(1,3)))];
aEst(1,:) = aRaw(1,:);

for i = 2:len
    if(i < 25)
        avPreviousRaw = avRaw(i-1,:);
    else
        avPreviousRaw = mean(avRaw(i-20:i-1,:));
    end
    timePeriod = seconds(t(i) - t(i-1));
%     timePeriod = (t(i) - t(i-1));
    [aEst(i,:),anglesPrev] = findEstimate(avRaw(i,:), aEst(i-1,:),...
        avPreviousRaw, anglesPrev, gMean, timePeriod);
end

aAdjusted = zeros(len,3,'double');
weightGyro2 = zeros(len,1,'double');
for i = 1:len
    if(sum(abs(avRaw(i,:))) < 0.2)
        weightGyro2(i) = 0;
    else
        weightGyro2(i) = weightGyro;
    end
    aAdjusted(i,:) = (aRaw(i,:) + aEst(i,:) * weightGyro2(i)) / (1 + weightGyro2(i));
end
% aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);
subplot(2,2,1);
plot(t,aAdjusted);
title('Acc Gyro Weight Adjusted');
legend('x','y','z');
subplot(2,2,2);
plot(t,aEst);
title('Gyro Estimated DCs');
legend('x','y','z');
subplot(2,2,3);
plot(t,aRaw);
title('Acc Raw');
legend('x','y','z');
subplot(2,2,4);
plot(t,avRaw);
title('Angular Velocity Raw');
legend('x','y','z');

gSph = zeros(len,3,'double');
gSphDegree = zeros(len,3,'double');
[gSph(:,1),gSph(:,2),gSph(:,3)] = cart2sph(aAdjusted(:,1),aAdjusted(:,2),aAdjusted(:,3));
gSphDegree(:,1:2) = rad2deg(gSph(:,1:2));
figure;
subplot(2,2,1);
plot(t,gSphDegree(:,1:2));
title('Theta Phi of DCs');
legend('Theta','Phi');

gVector = zeros(len,3,'double');
gSph(:,3) = gMean;
[gVector(:,1),gVector(:,2),gVector(:,3)] = sph2cart(gSph(:,1),gSph(:,2),gSph(:,3));
subplot(2,2,2);
plot(t,gVector);
title('Gravity Vector Cartesian');
legend('x','y','z');
aMotion = aRaw - gVector;
subplot(2,2,3);
plot(t,aRaw0);
title(' Purely Raw Acceleration (m/sec^2)')
legend('x','y','z');

% anglesDC = zeros(len,3,'double');
% aEstR = sqrt(aEst(:,1).^2 + aEst(:,2).^2 + aEst(:,3).^2);
anglesDC = rad2deg(abs(acos(aEst)));
subplot(2,2,4);
plot(t,anglesDC);
title('Angles of DCs with each axis');
legend('x','y','z');

timeInS = zeros(len,1,'double');
for i = 2:len
    timeInS(i) = timeInS(i-1) + seconds(t(i) - t(i-1));
end

findDisplacement(aMotion, weightGyro2, timeInS);

function [aEst,angles] = findEstimate(avCurrentRaw, aPreviousEst,...
        avPreviousRaw, anglesPrev, gMean, timePeriod)
%     timePeriod = 0.0022;
    aEst = zeros(1,3,'double');    
    avXZAvg = mean([avCurrentRaw(2),avPreviousRaw(2)]);
    angleXZcurr = anglesPrev(1) + avXZAvg * timePeriod;
    avYZAvg = mean([avCurrentRaw(1),avPreviousRaw(1)]);
    angleYZcurr = anglesPrev(2) + avYZAvg * timePeriod;
    angles = [angleXZcurr,angleYZcurr];
    
%     aEst(1) =  1  / sqrt(1  +   cotd(angleXZcurr)^2 * secd(angleYZcurr)^2 );
%     aEst(2) =  1  / sqrt(1  +   cotd(angleYZcurr)^2 * secd(angleXZcurr)^2 );
    aEst(1) = sind(angleXZcurr) / sqrt(1 + (cosd(angleXZcurr)^2)*(tand(angleYZcurr)^2));
    aEst(2) = sind(angleYZcurr) / sqrt(1 + (cosd(angleYZcurr)^2)*(tand(angleXZcurr)^2));
    aEst(3) =  sign(aPreviousEst(3)) * sqrt(1 - aEst(2)^2 - aEst(3)^2);
end

function findDisplacement(aMotion, weightGyro2, t)
    magNoG = aMotion;
    time = t;
    figure;
    [dataSize,~] = size(aMotion);
    for i = 1:3
        magNoG(:,i) = wden(aMotion(:,i),'modwtsqtwolog','s','mln',8,'sym4');
        magNoG(:,i) = magNoG(:,i) - mean(magNoG(1:500,i));
        
        thresh = max(abs(magNoG(1:500,i)));
        for j = 1:floor(dataSize/10)
            startAcc = ((j-1)*10) + 1;
            endAcc = j*10;
            if(max(abs(magNoG(startAcc:endAcc,i))) < thresh)
                magNoG(startAcc:endAcc,i) = 0;
            end
        end
        startAcc = (j*10) + 1;
        endAcc = dataSize;
        if(max(abs(magNoG(startAcc:endAcc,i))) < thresh)
            magNoG(startAcc:endAcc,i) = 0;
        end
    end
    
    subplot(2,2,3)
    plot(time,magNoG)
    xlabel('Time (sec)')
    ylabel('Minus G Acceleration (m/sec^2)')
    legend('x','y','z');
    
    subplot(2,2,1)
    plot(time,aMotion)
    xlabel('Time (sec)')
    ylabel('Acc Motion Vector Cartesian');
    legend('x','y','z');
    
    % First Integration (Acceleration - Veloicty)
    velmagNoG = cumtrapz(time,magNoG);
    for i = 1:dataSize
        if(weightGyro2(i) == 0)
            velmagNoG(i,:) = [0,0,0];
        end
    end
    subplot(2,2,2)
    plot(time,velmagNoG);
    xlabel('Time (sec)')
    ylabel('Velocity (m/sec)')
    legend('x','y','z');    
    
    % Second Integration (Velocity - Displacement)
    DisplacementmagNoG=cumtrapz(time, velmagNoG);
    subplot(2,2,4)
    plot(time,DisplacementmagNoG);
    xlabel('Time (sec)')
    ylabel('Displacement (m)')
    legend('x','y','z');
    
    TotalDisplacement = norm(DisplacementmagNoG(dataSize,:));
    disp('Total Displcement(m) = ');
    disp(TotalDisplacement * 9.8);
end