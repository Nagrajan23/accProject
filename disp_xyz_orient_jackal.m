close all;

% importRaspPi;
weightGyro = 5;
avRawSumThresh = 0.34;
% gMultiplier = gMultiplier;
gMultiplier = 0.195;

% t = Untitled.Time(1:len);
[len,~] = size(imuData);
aRaw0 = zeros(len,3,'double');
avRaw0 = zeros(len,3,'double');
t = zeros(len,1,'double');
for i = 1:len
    aRaw0(i,:) = [imuData{i}.LinearAcceleration.X,...
        imuData{i}.LinearAcceleration.Y, imuData{i}.LinearAcceleration.Z];
    avRaw0(i,:) = [imuData{i}.AngularVelocity.X,... 
        imuData{i}.AngularVelocity.Y, imuData{i}.AngularVelocity.Z];
    t(i) = imuData{i}.Header.Stamp.Sec +...
        imuData{i}.Header.Stamp.Nsec/1e09;
end    

% [lenA,~] = size(avRaw0);
% [lenAv,~] = size(aRaw0);
% len = min(lenA,lenAv);

% aRaw0 = aRaw0(1:len,:);
% avRaw0 = avRaw0(1:len,:);
t = t(1:len);

timeInS = zeros(len,1,'double');
for i = 2:len
    timeInS(i) = timeInS(i-1) + (t(i) - t(i-1));
end

avRaw = avRaw0;
aRaw1 = aRaw0;
aRaw = aRaw0;

figure (1);
subplot(3,2,1);
plot(timeInS,aRaw0/9.8);
title('Raw Acceleration');
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Acceleration(g's)");
ylim([-2 2]);


figure (1);
subplot(3,2,2);
plot(timeInS,avRaw0);
title('Raw Angular Velocity');
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Angular Velocity(rad/s)");
ylim([-50 50]);

% Loop for filtering all axis of raw data
for i = 1:3
    % De-noise noisy signal using minimax threshold with 
    % a multiple level estimation of noise standard deviation.
    avRaw(:,i) = wden(avRaw0(:,i),'modwtsqtwolog','s','mln',8,'sym4');
%     aRaw1(:,i) = wden(aRaw0(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    figure (1);
    subplot(3,2,4);
    plot(timeInS,avRaw);
    title('De-noised Angular Velocity');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Angular Velocity(rad/s)");
    ylim([-50 50]);
%     subplot(4,1,2);
%     plot(t,aRaw1);
    
%     title('De-noised Acceleration');
%     legend('x','y','z');
    
    
    order = 6; % 6th Order Filter
    [b1,a1] = butter(order, 0.01, 'low');
    aRaw(:,i) = filtfilt(b1 ,a1 , aRaw1(:,i));
    
    figure (1);
    subplot(3,2,3);
    plot(timeInS,aRaw/9.8);
    title('Low Pass Butterworth Filter (Acceleration)');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Acceleration(g's)");
    ylim([-2 2]);
    
    aRaw1(:,i) = medfilt1(aRaw1(:,i),100);
    aRaw1(1,i) = aRaw0(1,i);
    
    figure (1);
    subplot(3,2,5);
    plot(timeInS,aRaw1/9.8);
    title('Median Filter (Acceleration)');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Acceleration(g's)");
    ylim([-2 2]);
    
    avRaw(:,i) = avRaw(:,i) - mean(avRaw(1:300,i));
    avRaw(:,i) = medfilt1(avRaw(:,i),100);
    
    figure (1);
    subplot(3,2,6);
    plot(timeInS,avRaw);
    title('Median Filter (Angular Velocity)');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Angular Velocity(rad/s)");
    ylim([-50 50]);
end

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
    timePeriod = (t(i) - t(i-1));
    
    [aEst(i,:),anglesPrev] = findEstimate(avRaw(i,:), aEst(i-1,:),...
        avPreviousRaw, anglesPrev, gMean, timePeriod);
end

aAdjusted = zeros(len,3,'double');
weightGyro2 = zeros(len,1,'double');
for i = 1:len
%     if(sum(abs(avRaw(i,:))) < avRawSumThresh)
%         weightGyro2(i) = 0;
%     else
        weightGyro2(i) = weightGyro;
%     end
    aAdjusted(i,:) = (aRaw(i,:) + aEst(i,:) * weightGyro2(i)) / (1 + weightGyro2(i));
end
% aAdjusted = (aRaw + aEst * weightGyro) / (1 + weightGyro);

figure (2);
subplot(3,2,1);
plot(timeInS,aAdjusted/9.8);
title('Acc Gyro Weight Adjusted');
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Unit Vector (g's)")
ylim([-0.5 1.5]);
figure (2);
subplot(3,2,2);
plot(timeInS,aEst/9.8);
title('Gyro Estimated DCs');
legend('x','y','z');
xlabel('Time (sec)')
ylim([-0.5 1.5]);
ylabel("Unit Vector (g's)");
% figure (2);
% subplot(2,2,3);
% plot(t,aRaw);
% title('Acc Raw');
% legend('x','y','z');
% subplot(2,2,4);
% plot(t,avRaw);
% title('Angular Velocity Raw');
% legend('x','y','z');

gSph = zeros(len,3,'double');
gSphDegree = zeros(len,3,'double');
[gSph(:,1),gSph(:,2),gSph(:,3)] = cart2sph(aAdjusted(:,1),aAdjusted(:,2),aAdjusted(:,3));
gSphDegree(:,1:2) = rad2deg(gSph(:,1:2));
figure (2);
subplot(3,2,3);
plot(timeInS,gSphDegree(:,1:2));
title('Theta Phi of DCs');
legend('Azimuth Angle','Polar Angle');
xlabel('Time (sec)')
ylabel({"Azimuth and"; "Polar Angles"; "(Degree's)"});

gVector = zeros(len,3,'double');
gSph(:,3) = gMean;
[gVector(:,1),gVector(:,2),gVector(:,3)] = sph2cart(gSph(:,1),gSph(:,2),gSph(:,3));
figure (2);
subplot(3,2,4);
plot(timeInS,gVector/9.8);
title('Gravity Vector Cartesian');
legend('x','y','z');
ylim([-0.5 1.5]);
xlabel('Time (sec)')
ylabel("Acceleration(g's)");
aMotion = aRaw - gVector;
% subplot(2,2,3);
% plot(t,aRaw0);
% title(' Purely Raw Acceleration (m/sec^2)')
% legend('x','y','z');

% anglesDC = zeros(len,3,'double');
% aEstR = sqrt(aEst(:,1).^2 + aEst(:,2).^2 + aEst(:,3).^2);
anglesDC = rad2deg(abs(acos(aEst)));
figure (2);
subplot(3,2,5);
plot(timeInS,anglesDC);
title('Angles of DCs with each axis');
legend('x','y','z');
xlabel('Time (sec)')
ylabel({"Euler Angles ";"(Degree's)"});


figure (2);
subplot(3,2,6);
plot(timeInS,aMotion);
title('Dynamic Motion');
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Acceleration(g's)");

DisplacementmagNoG = findDisplacement(aMotion, weightGyro2, timeInS);


[lenPos,~] = size(odomfiltData);
pos1 = [odomfiltData{1}.Pose.Pose.Position.X,...
    odomfiltData{1}.Pose.Pose.Position.Y,odomfiltData{1}.Pose.Pose.Position.Z];
pos2 = [odomfiltData{lenPos}.Pose.Pose.Position.X,...
    odomfiltData{lenPos}.Pose.Pose.Position.Y,odomfiltData{lenPos}.Pose.Pose.Position.Z];
odomDistance = norm([pos1;pos2])

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
    aEst(3) =  sign(aPreviousEst(3)) * sqrt(1 - aEst(1)^2 - aEst(2)^2);
    aEst(3) = 9.8;
end

function DisplacementmagNoG = findDisplacement(aMotion, weightGyro2, t)
    gMultiplier = 0.6989;
    magNoG = aMotion;
    time = t;
    [dataSize,~] = size(aMotion);
    for i = 1:3
        magNoG(:,i) = wden(aMotion(:,i),'modwtsqtwolog','s','mln',8,'sym4');
        magNoG(:,i) = magNoG(:,i) - mean(magNoG(1:500,i));
        
        thresh = max(abs(magNoG(1:500,i)));
        for j = 1:floor(dataSize/10)
            startAcc = ((j-1)*10) + 1;
            endAcc = j*10;
            if(max(abs(magNoG(startAcc:endAcc,i))) < thresh)
%                 magNoG(startAcc:endAcc,i) = 0;
            end
        end
        startAcc = (j*10) + 1;
        endAcc = dataSize;
        if(max(abs(magNoG(startAcc:endAcc,i))) < thresh)
%             magNoG(startAcc:endAcc,i) = 0;
        end
    end
   
    figure (3);
    subplot(2,2,1);
    plot(time,aMotion);
    xlabel('Time (sec)');
    ylabel('Dynamic Acceleration');
    legend('x','y','z');
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    title('Dynamic Acceleration');
   
    
    figure (3);
    subplot(2,2,3);
    magNoG1 = magNoG * gMultiplier;
    plot(time,magNoG1);
    xlabel('Time (sec)');
    ylabel('Filtered Dynamic Acceleration (m/sec^2)');
    legend('x','y','z');
    title('Filtered Dynamic Acceleration');
    
    
    % First Integration (Acceleration - Veloicty)
    velmagNoG = cumtrapz(time,magNoG);
    for i = 1:dataSize
        if(weightGyro2(i) == 0)
%             velmagNoG(i,:) = [0,0,0];
        end
    end
    velmagNoG(:,3)=0;
    figure (3);
    subplot(2,2,2);
    velmagNoG1 = velmagNoG * gMultiplier;
    %velmagNoG1(:,3)=0;
    plot(time,velmagNoG1);
    xlabel('Time (sec)');
    ylabel('Velocity (m/sec)');
    title('Velocity');
    legend('x','y','z');
    
    % Second Integration (Velocity - Displacement)
    DisplacementmagNoG=cumtrapz(time, velmagNoG);
    subplot(2,2,4);
    DisplacementmagNoG1 = DisplacementmagNoG * gMultiplier;
    plot(time,DisplacementmagNoG1);
    xlabel('Time (sec)')
    ylabel('Displacement (m)');
    legend('x','y','z');
    title('Displacement'); 
    
    TotalDisplacement = norm(DisplacementmagNoG(dataSize,:));
    disp('Total Displcement(m) = ');
    disp(TotalDisplacement * gMultiplier);
    
    figure (4);
    plot(DisplacementmagNoG1(:,1),DisplacementmagNoG1(:,2));
    hold on
    plot(DisplacementmagNoG1(1,1),DisplacementmagNoG1(1,2),'o','MarkerSize',10,'MarkerFaceColor','g')
    hold on
    %plot(DisplacementmagNoG1(length(DisplacementmagNoG1(:,2)),2),DisplacementmagNoG1(length(DisplacementmagNoG1(:,1)),1),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot(DisplacementmagNoG1(length(DisplacementmagNoG1(:,1)),1),DisplacementmagNoG1(length(DisplacementmagNoG1(:,2)),2),'o','MarkerSize',10,'MarkerFaceColor','b')
    title('2D Movement');
    xlabel('X-Axis (m)')
    ylabel("Y-Axis (m)");
    legend('Movement Pattern');
    grid on
    
%     figure (4);
%     x = DisplacementmagNoG1(:,1);
%     y = DisplacementmagNoG1(:,2);
%   scatter(x,y);
% % y = DisplacementmagNoG1(:,2);
%   plot(DisplacementmagNoG1(:,1),DisplacementmagNoG1(:,2));
end