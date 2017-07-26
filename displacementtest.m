%clc
%clear all
%close all

% Test Commit Nag 3
% Test Commit Nag 4
%Test 6

% We both add a line here - Nagrajan
%Hello Nag



% Now added line 14 - Nagrajan
% Now added line 15 - Nagrajan
% it's just too much
% Now added line 16 - Nagrajan
% Now added line 18 - Nagrajan
% It'sss just way too much
% It's just way to way much line 20

% Testing again 20 - Nagrajan

%time = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\time.txt')
%accx = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accx.txt')
%accy = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accy.txt')
%accz = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accz.txt')

% time = t(7563:10332);
time = t;
accx = a(:,1);
accy = a(:,2);
accz = a(:,3);
%  Hello to the world of legions
%  Hello to the world of warriors
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
figure
% plot(time,accx,'r',time,accy,'g',time,accz,'b')
% subplot(2,2,1)
% plot(time,accx);
% xlabel('Time (sec)')
% ylabel('Raw Acceleration (m/sec^2)')

mag = sqrt(sum(x.^2 + y.^2 + z.^2, 2));
% subplot(2,2,1)
% plot(time,mag);
% xlabel('Time (sec)')
% ylabel('Combined Raw Acceleration (m/sec^2)')

magNoG = mag - mean(mag);
% magNoG = mag;
% accx_meanned = accx - mean(accx);
% accx = magNoG;
% accy_meanned = accy - mean(accy);
% accy = accy_meanned;
% accz_meanned = accz - mean(accz);
% accz = accz_meanned;

% figure,plot(time,accx,'r',time,accy,'g',time,accz,'b')
subplot(2,2,1)
plot(time,magNoG,'r')
xlabel('Time (sec)')
ylabel('Minus mean Acceleration (m/sec^2)')

%% Design High Pass Filter

fs = 10; % Sampling Rate

fc = 0.1/36; % Cut off Frequency

order = 6; % 6th Order Filter

%% Filter Acceleration Signals

[b1 a1] = butter(order,fc,'low');

magNoGf=filtfilt(b1,a1,magNoG);
% accxf = accx;
% accyf=filtfilt(b1,a1,accy);
% acczf=filtfilt(b1,a1,accz);

% figure (2)

% plot(time,accxf,'r',time,accyf,'g',time,acczf,'b');
subplot(2,2,3)
plot(time,magNoGf);
xlabel('Time (sec)')
ylabel('Filtered Acceleration (m/sec^2)')

%% First Integration (Acceleration - Veloicty)

velocitymagNoG=cumtrapz(time,magNoGf);
% velocityy=cumtrapz(time,accyf);
% velocityz=cumtrapz(time,acczf);

% figure (3)

% plot(time,velocityx,'r',time,velocityy,'g',time,velocityz,'b')
subplot(2,2,2)
plot(time,velocitymagNoG);
xlabel('Time (sec)')

ylabel('Velocity (m/sec)')

%% Filter Veloicty Signals

[b2 a2] = butter(order,fc,'low');

velmagNoGf = velocitymagNoG;
% velxf = filtfilt(b2,a2,velocityx);
% velyf = filtfilt(b2,a2,velocityy);
% velzf = filtfilt(b2,a2,velocityz);

%% Second Integration (Velocity - Displacement)

DisplacementmagNoG=cumtrapz(time, velmagNoGf);
% Displacementy=cumtrapz(time, velyf);
% Displacementz=cumtrapz(time, velzf);
subplot(2,2,4)
% plot(time,Displacementx,'r',time,Displacementy,'g',time,Displacementz,'b')
plot(time,DisplacementmagNoG);
xlabel('Time (sec)')
ylabel('Displacement (m)')

