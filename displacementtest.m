%clc
%clear all
close all
% hello to Touqeer
%time = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\time.txt')
%accx = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accx.txt')
%accy = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accy.txt')
%accz = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accz.txt')

[b1,a1] = butter(6,0.01,'high');
[dataSize,~] = size(a);
for i = 1:3
    a(:,i)=filtfilt(b1,a1,a(:,i));
    thresh = max(abs(a(1:300,i)))
    for j = 1:floor(dataSize/10)
        startAcc = ((j-1)*10) + 1;
        endAcc = j*10;
        if(max(abs(a(startAcc:endAcc,i))) < thresh)
            a(startAcc:endAcc,i) = 0;
        end
    end
    startAcc = (j*10) + 1;
    endAcc = dataSize;
    if(max(abs(a(startAcc:endAcc,i))) < thresh)
        a(startAcc:endAcc,i) = 0;
    end
end

% time = t(7563:10332);
time = t;
accx = a(:,1);
accy = a(:,2);
accz = a(:,3);

% aa = a(7563:10332,:);
% accx = aa(:,1);
% accy = aa(:,2);
% accz = aa(:,3);

% testlac = test3lac(:,:);
% time = testlac.time_tick;
% accx = testlac.lac_X_value;
% accy = testlac.lac_Y_value;
% accz = testlac.lac_Z_value;
x=accx;
y=accy;
z=accz;
subplot(3,1,1)
plot(time,x,'r');
subplot(3,1,2)
plot(time,y,'g');
subplot(3,1,3)
plot(time,z,'b');
% plot(time,accx,'r',time,accy,'g',time,accz,'b')
% subplot(2,2,1)
% plot(time,accx);
% xlabel('Time (sec)')
% ylabel('Raw Acceleration (m/sec^2)')

% mag = sqrt(sum(x.^2 + y.^2 + z.^2, 2));
mag = sum(x + y + z, 2);
% subplot(2,2,1)
% plot(time,mag);
% xlabel('Time (sec)')
% ylabel('Combined Raw Acceleration (m/sec^2)')

% magNoG = mag - mean(mag(1:200));
% magNoG = mag;
% accx_meanned = accx - mean(accx);
% accx = magNoG;
% accy_meanned = accy - mean(accy);
% accy = accy_meanned;
% accz_meanned = accz - mean(accz);
% accz = accz_meanned;

magFFT = fft(mag);
[b1,a1] = butter(6,0.01,'high');
magNoG=filtfilt(b1,a1,mag);

for i = 1:floor(dataSize/10)
    startMag = ((i-1)*10) + 1;
    endMag = i*10;
    if(max(abs(magNoG(startMag:endMag))) < 0.1)
        magNoG(startMag:endMag) = 0;
    end
end
startMag = (i*10) + 1;
endMag = dataSize;
if(max(abs(magNoG(startMag:endMag))) < 0.1)
    magNoG(startMag:endMag) = 0;
end

% figure,plot(time,accx,'r',time,accy,'g',time,accz,'b')
figure;
subplot(2,2,1)
plot(time,magNoG,'r')
xlabel('Time (sec)')
ylabel('Minus mean Acceleration (m/sec^2)')

%% High Pass Filter Paramters

fs = 10; % Sampling Rate

fc = 0.1/36; % Cut off Frequency
iterFc = 1;

order = 6; % 6th Order Filter

%% Filter Acceleration Signals
distVariation = zeros(iterFc,1,'double');
for i = 1:iterFc
    i
    fc = 0.01/i;
%     [b1,a1] = butter(order,[0.016 0.05],'bandpass');
      [b1,a1] = butter(order,[0.016 0.05],'bandpass');
%     [b1,a1] = butter(order,0.1/36,'high');
%     [b1,a1] = butter(order,fc,'low');
%     magNoGf=filtfilt(b1,a1,magNoG);
%     magNoGf = filter(Hbp2,magNoG);
%       [b1,a1] = butter(order,[0.01 0.05],'bandpass');
%     [b1,a1] = butter(order,0.1/36,'high');
%     [b1,a1] = butter(order,fc,'low');
%     magNoGf=filtfilt(b1,a1,magNoG);
% xdMODWT = wden(XN,'modwtsqtwolog','s','mln',4,'sym4');
% magNoGf=filtfilt(b1,a1,magNoG);

% Wavelet filtering
% magNoGf = wden(magNoGf,'modwtsqtwolog','s','mln',8,'sym4');
%     magNoGf = filter(Hlp,magNoG);
    magNoGf = magNoG;
    % plot(time,accxf,'r',time,accyf,'g',time,acczf,'b');

%     Kalman Filter - 27 Jun
    initial_estimate = 0;
    estErr = 10^-2;
    mErr = 0.0024;
    inputVals = magNoGf;
    estPrev = initial_estimate;
    [~,len] = size(inputVals);

%     subplot(2,2,1)
%     if iterFc == 1
%         plot(time,magNoGf);
%         xlabel('Time (sec)')
%         ylabel('Acceleration (m/sec^2)')
%     end
    
    for i = 1:size(inputVals)
%         i
        kalmanGain = estErr / (estErr + mErr);
        estCur = estPrev + kalmanGain * (inputVals(i) - estPrev);
        estErr = (1 - kalmanGain) * estErr;
        estPrev = estCur;
%         if i > size(inputVals)/10
%             magNoGf(i) = estCur;
%         end
%         estCur
%         estErr
    end
    
    
%     Touqueer Code commented - Raj 2607
%     hoursPerDay = 34060;
%     coeff24hMA = ones(1, hoursPerDay)/hoursPerDay;
%     magNoGf =filter(coeff24hMA,1,magNoG)

    subplot(2,2,3)
    if iterFc == 1
        plot(time,magNoGf);
        xlabel('Time (sec)')
        ylabel('Filtered Acceleration (m/sec^2)')
    end

    %% First Integration (Acceleration - Veloicty)

    velocitymagNoG=cumtrapz(time,magNoGf);
    % velocityy=cumtrapz(time,accyf);
    % velocityz=cumtrapz(time,acczf);

    % figure (3)

    % plot(time,velocityx,'r',time,velocityy,'g',time,velocityz,'b')

    %% Filter Veloicty Signals

%     [b2,a2] = butter(order,fc,'low');
    [b2,a2] = butter(order,[0.016 0.022],'bandpass');

    velmagNoGf = velocitymagNoG;
    [velSize,~] = size(velocitymagNoG);
    for i = 1:floor(velSize/10)
        startVel = ((i-1)*10) + 1;
        endVel = i*10;
        if(max(abs(velmagNoGf(startVel:endVel))) < 0.1)
            velmagNoGf(startVel:endVel) = 0;
        end
    end
    startVel = (i*10) + 1;
    endVel = velSize;
    if(max(abs(velmagNoGf(startVel:endVel))) < 0.1)
        velmagNoGf(startVel:endVel) = 0;
    end
    
    subplot(2,2,2)
    if iterFc == 1
        plot(time,velmagNoGf);
        xlabel('Time (sec)')
        ylabel('Velocity (m/sec)')
    end
%     velmagNoGf = filtfilt(b2,a2,velocitymagNoG);
    % velyf = filtfilt(b2,a2,velocityy);
    % velzf = filtfilt(b2,a2,velocityz);

    %% Second Integration (Velocity - Displacement)

    DisplacementmagNoG=cumtrapz(time, velmagNoGf);
    [r,c] = size(DisplacementmagNoG);
    distVariation(i) = DisplacementmagNoG(r);
end

% Displacementy=cumtrapz(time, velyf);
% Displacementz=cumtrapz(time, velzf);
subplot(2,2,4)
% plot(time,Displacementx,'r',time,Displacementy,'g',time,Displacementz,'b')
plot(time,DisplacementmagNoG);
xlabel('Time (sec)')
ylabel('Displacement (m)')
if iterFc > 1
    figure, plot(1:100,distVariation);
end

