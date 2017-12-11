%% Section to Variables Initialization
close all;
clear

%To get data in m/s^2
gMultiplier = 0.53; 
%Initialzing weight factor for complimentory filter for Angular velocity
weightGyro = 5; 

%% Section for MPU 9255 data
% Make sure to uncomment the following line or run it once to get data
%from the raspberry pi or have Untitled variable containing data in workspace

MPU9255_data
aRaw0 = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
avRaw0 = [Untitled.Gyroscope_x,Untitled.Gyroscope_y,Untitled.Gyroscope_z];

[lenA,~] = size(avRaw0);
[lenAv,~] = size(aRaw0);
len = min(lenA,lenAv);
t = Untitled.Time(1:len);

%To get the same length of data for both accelerometer and gyroscope
aRaw0 = aRaw0(1:len,:);
avRaw0 = avRaw0(1:len,:);
t = t(1:len);

timeInS = zeros(len,1,'double');
%Converting absolute time to time series in seconds for MPU9255
for i = 2:len
    if(exist('a','var') == 1)
        timeInS(i) = timeInS(i-1) + (t(i) - t(i-1));
    else
        timeInS(i) = timeInS(i-1) + seconds(t(i) - t(i-1));
    end
end

%Thershold for zero velocity on Angular velocity
avRawSumThresh = 0.34;

%flag to know the device type
selection_flag = 1

%% Section for the Jackal Data
% Make sure to uncomment the following line to run to get data
%from the Jackal Robot or  have imuData in workspace

%{
Jackal_Data
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

t = t(1:len);
%Converting absolute time to time series in seconds for the Jackal Robot
timeInS = zeros(len,1,'double');
for i = 2:len
    timeInS(i) = timeInS(i-1) + (t(i) - t(i-1));
end 

%Thershold for zero velocity on Angular velocity
avRawSumThresh = 0.34;

%flag to know the device type
selection_flag = 2
%}
%% Section for the iPhone 7 Data

%{

load('Lib311_0912_1716.mat')

aRaw0 = a;
aRawNormal = normr(a);
avRaw0 = rad2deg(av);

[lenA,~] = size(avRaw0);
[lenAv,~] = size(aRaw0);
len = min(lenA,lenAv);

%To get the same length of data for both accelerometer and gyroscope
aRaw0 = aRaw0(1:len,:);
avRaw0 = avRaw0(1:len,:);
t = t(1:len);

timeInS = zeros(len,1,'double');
for i = 2:len
    if(exist('a','var') == 1)
        timeInS(i) = timeInS(i-1) + (t(i) - t(i-1));
    else
        timeInS(i) = timeInS(i-1) + seconds(t(i) - t(i-1));
    end
end

%Thershold for zero velocity on Angular velocity for iPhone 7
avRawSumThresh = 0.5;

%flag to know the device type
selection_flag = 3
%}
%% Filtering Section

avRaw = avRaw0;
aRaw1 = aRaw0;
aRaw = aRaw0;

%Plotting Raw data (Accelerometer) without any changes to data
figure (1);
subplot(3,2,1);
if(selection_flag == 2)
        plot(timeInS,aRaw0/9.8);
    else
        plot(timeInS,aRaw0);
end
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
    
    figure (1);
    subplot(3,2,4);
    plot(timeInS,avRaw);
    title('De-noised Angular Velocity');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Angular Velocity(rad/s)");
    ylim([-50 50]);
        
    order = 6; % 6th Order Filter
    [b1,a1] = butter(order, 0.01, 'low');
    aRaw(:,i) = filtfilt(b1 ,a1 , aRaw1(:,i));
    
    figure (1);
    subplot(3,2,3);
    if(selection_flag == 2)
        plot(timeInS,aRaw/9.8);
    else
        plot(timeInS,aRaw);
    end
    title('Low Pass Butterworth Filter (Acceleration)');
    legend('x','y','z');
    xlabel('Time (sec)')
    ylabel("Acceleration(g's)");
    ylim([-2 2]);
   
    aRaw1(:,i) = medfilt1(aRaw1(:,i),100);
    aRaw1(1,i) = aRaw0(1,i);
   
    figure (1);
    subplot(3,2,5);
    if(selection_flag == 2)
        plot(timeInS,aRaw1/9.8);
    else
        plot(timeInS,aRaw1);
    end
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
%% Orientation, Gravity Vector, and Dynamic accleration Calculation
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
    
    if(selection_flag ~= 1)
        timePeriod = (t(i) - t(i-1));
    else
        timePeriod = seconds(t(i) - t(i-1));
    end
    
    [aEst(i,:),anglesPrev] = findEstimate(avRaw(i,:), aEst(i-1,:),...
        avPreviousRaw, anglesPrev, gMean, timePeriod);
end

aAdjusted = zeros(len,3,'double');
weightGyro2 = zeros(len,1,'double');

%Thershold on gyroweightage according to anuglar velocity
for i = 1:len
    if(selection_flag == 2)
        weightGyro2(i) = weightGyro;
    else
        if(sum(abs(avRaw(i,:))) < avRawSumThresh)
            weightGyro2(i) = 0;
        else
            weightGyro2(i) = weightGyro;
        end
    end
    aAdjusted(i,:) = (aRaw(i,:) + aEst(i,:) * weightGyro2(i)) / (1 + weightGyro2(i));
end

figure (2);
subplot(3,2,1);
if(selection_flag == 2)
    plot(timeInS,aAdjusted/9.8);
else
    plot(timeInS,aAdjusted);
end
title('Acc Gyro Weight Adjusted');
legend('x','y','z');
ylim([-0.5 1.5]);
ylabel("Unit Vector (g's)")
xlabel('Time (sec)')

subplot(3,2,2);
if(selection_flag == 2)
    plot(timeInS,aEst/9.8);
else
    plot(timeInS,aEst);
end
title('Gyro Estimated DCs');
ylim([-0.5 1.5]);
legend('x','y','z');
ylabel("Unit Vector (g's)")
xlabel('Time (sec)')

gSph = zeros(len,3,'double');
gSphDegree = zeros(len,3,'double');
[gSph(:,1),gSph(:,2),gSph(:,3)] = cart2sph(aAdjusted(:,1),aAdjusted(:,2),aAdjusted(:,3));
%Radians to Degrees
gSphDegree(:,1:2) = rad2deg(gSph(:,1:2));

subplot(3,2,3);
plot(timeInS,gSphDegree(:,1:2));
title('Theta Phi of DCs');
legend('Azimuth Angle','Polar Angle');
xlabel('Time (sec)')
ylabel({"Azimuth and"; "Polar Angles"; "(Degree's)"});

gVector = zeros(len,3,'double');
gSph(:,3) = gMean;

%Gravity vector
[gVector(:,1),gVector(:,2),gVector(:,3)] = sph2cart(gSph(:,1),gSph(:,2),gSph(:,3));

%Calculating Dynamic motion from filtered acceleration
aMotion = aRaw - gVector;
anglesDC = rad2deg(abs(acos(aEst)));

subplot(3,2,4);
if(selection_flag == 2)
    plot(timeInS,gVector/9.8);
else
    plot(timeInS,gVector);
end

title('Gravity Vector Cartesian');
ylim([-0.5 1.5]);
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Acceleration(g's)");

subplot(3,2,5);
plot(timeInS,anglesDC);
title('Angles of DCs with each axis');
legend('x','y','z');
xlabel('Time (sec)')
ylabel({"Euler Angles ";"(Degree's)"});

subplot(3,2,6);
if(selection_flag == 2)
    plot(timeInS,aMotion/9.8);
else
    plot(timeInS,aMotion);
end
title('Dynamic Motion');
legend('x','y','z');
xlabel('Time (sec)')
ylabel("Acceleration(g's)");

DisplacementmagNoG = findDisplacement(aMotion, weightGyro2, timeInS,gMultiplier);

if(selection_flag == 2)
        [lenPos,~] = size(odomfiltData);
    pos1 = [odomfiltData{1}.Pose.Pose.Position.X,...
        odomfiltData{1}.Pose.Pose.Position.Y,odomfiltData{1}.Pose.Pose.Position.Z];
    pos2 = [odomfiltData{lenPos}.Pose.Pose.Position.X,...
        odomfiltData{lenPos}.Pose.Pose.Position.Y,odomfiltData{lenPos}.Pose.Pose.Position.Z];
    odomDistance = norm([pos1;pos2])
end

%Function to estimatet the Orientation and Gravity vector
function [aEst,angles] = findEstimate(avCurrentRaw, aPreviousEst,...
        avPreviousRaw, anglesPrev, gMean, timePeriod)
    aEst = zeros(1,3,'double');    
    avXZAvg = mean([avCurrentRaw(2),avPreviousRaw(2)]);
    angleXZcurr = anglesPrev(1) + avXZAvg * timePeriod;
    avYZAvg = mean([avCurrentRaw(1),avPreviousRaw(1)]);
    angleYZcurr = anglesPrev(2) + avYZAvg * timePeriod;
    angles = [angleXZcurr,angleYZcurr];
    
    aEst(1) = sind(angleXZcurr) / sqrt(1 + (cosd(angleXZcurr)^2)*(tand(angleYZcurr)^2));
    aEst(2) = sind(angleYZcurr) / sqrt(1 + (cosd(angleYZcurr)^2)*(tand(angleXZcurr)^2));
    aEst(3) =  sign(aPreviousEst(3)) * sqrt(1 - aEst(1)^2 - aEst(2)^2);
end


%% Section for Velocity and displacement profile 

%Function to calculate the velocity and position profiles
function DisplacementmagNoG = findDisplacement(aMotion, weightGyro2, t,gMultiplier)
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
                magNoG(startAcc:endAcc,i) = 0;
            end
        end
        startAcc = (j*10) + 1;
        endAcc = dataSize;
        if(max(abs(magNoG(startAcc:endAcc,i))) < thresh)
            magNoG(startAcc:endAcc,i) = 0;
        end
    end
   
    magNoG1 = magNoG * gMultiplier;

    figure (3);
    subplot(2,2,1);
    plot(time,aMotion);
    title('Dynamic Acceleration');
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    legend('x','y','z');
    subplot(2,2,3);
    plot(time,magNoG1);
    title('Filtered Dynamic Acceleration');
    xlabel('Time (sec)');
    ylabel('Acceleration (m/s^2)');
    legend('x','y','z');
    
    % First Integration (Acceleration - Veloicty)
    velmagNoG = cumtrapz(time,magNoG);
    for i = 1:dataSize
        if(weightGyro2(i) == 0)
            velmagNoG(i,:) = [0,0,0];
        end
    end
    
    velmagNoG1 = velmagNoG * gMultiplier;
    
    subplot(2,2,2);
    plot(time,velmagNoG1);
    title('Velocity');
    xlabel('Time (sec)');
    ylabel('Velocity (m/s)');
    legend('x','y','z');    
    
    % Second Integration (Velocity - Displacement)
    DisplacementmagNoG=cumtrapz(time, velmagNoG);
    DisplacementmagNoG1 = DisplacementmagNoG * gMultiplier;
    
    subplot(2,2,4);
    plot(time,DisplacementmagNoG1);
    title('Displacement');    
    xlabel('Time (sec)')
    ylabel('Displacement (m)');
    legend('x','y','z');
    
    TotalDisplacement = norm(DisplacementmagNoG(dataSize,:));
    disp('Total Displcement(m) = ');
    disp(TotalDisplacement * gMultiplier);
    
    d = hypot(diff(DisplacementmagNoG(:,1)), diff(DisplacementmagNoG(:,2)));
    TotalDistance = sum(d); 
    disp('Total Distance(m) = ');
    disp(TotalDistance * gMultiplier);
    
    %Plot for Movement Pattern on Cartesian plane
    figure(4);
    plot(DisplacementmagNoG1(:,1),DisplacementmagNoG1(:,2));
    hold on
    plot(DisplacementmagNoG1(1,1),DisplacementmagNoG1(1,2),'o','MarkerSize',10,'MarkerFaceColor','g')
    hold on
    plot(DisplacementmagNoG1(length(DisplacementmagNoG1(:,1)),1),DisplacementmagNoG1(length(DisplacementmagNoG1(:,2)),2),'o','MarkerSize',10,'MarkerFaceColor','b')
    title('2D Movement');
    xlabel('X-Axis (m)')
    ylabel("Y-Axis (m)");
    legend('Movement Pattern');
    grid on
end