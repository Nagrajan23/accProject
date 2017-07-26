% Nagrajan Test1
%clc
%clear all
% close all

%time = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\time.txt')
%accx = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accx.txt')
%accy = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accy.txt')
%accz = load('C:\Users\touqe\OneDrive\Documents\MATLAB\projecttest\accz.txt')

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
figure
% plot(time,accx,'r',time,accy,'g',time,accz,'b')
plot(time,accx);
xlabel('Time (sec)')
ylabel('Acceleration (m/sec^2)')

mag = sqrt(sum(x.^2 + y.^2 + z.^2, 2));
magNoG = mag - mean(mag);
% magNoG = mag;
accx_meanned = accx - mean(accx);
accx = magNoG;
accy_meanned = accy - mean(accy);
accy = accy_meanned;
accz_meanned = accz - mean(accz);
accz = accz_meanned;

figure,plot(time,accx,'r',time,accy,'g',time,accz,'b')
xlabel('Time (sec)')
ylabel('Acceleration (m/sec^2)')

%% Design High Pass Filter

fs = 10; % Sampling Rate

fc = 0.1/36; % Cut off Frequency

order = 6; % 6th Order Filter

%% Filter Acceleration Signals

[b1 a1] = butter(order,fc,'low');

accxf=filtfilt(b1,a1,accx);
% accxf = accx;
accyf=filtfilt(b1,a1,accy);
acczf=filtfilt(b1,a1,accz);

% figure (2)

% plot(time,accxf,'r',time,accyf,'g',time,acczf,'b'); 
figure,plot(time,accxf);
xlabel('Time (sec)')

ylabel('Acceleration (m/sec^2)')

%% First Integration (Acceleration - Veloicty)

velocityx=cumtrapz(time,accxf);
velocityy=cumtrapz(time,accyf);
velocityz=cumtrapz(time,acczf);

% figure (3)

% plot(time,velocityx,'r',time,velocityy,'g',time,velocityz,'b')
figure,plot(time,velocityx);
xlabel('Time (sec)')

ylabel('Velocity (m/sec)')

%% Filter Veloicty Signals

[b2 a2] = butter(order,fc,'low');

velxf = velocityx;
% velxf = filtfilt(b2,a2,velocityx);
velyf = filtfilt(b2,a2,velocityy);
velzf = filtfilt(b2,a2,velocityz);

%% Second Integration (Velocity - Displacement)

Displacementx=cumtrapz(time, velxf);
Displacementy=cumtrapz(time, velyf);
Displacementz=cumtrapz(time, velzf);
figure(4)

% plot(time,Displacementx,'r',time,Displacementy,'g',time,Displacementz,'b')
plot(time,Displacementx);
xlabel('Time (sec)')

ylabel('Displacement (m)')

