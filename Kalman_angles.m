% close all

aRaw0 = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
avRaw0 = [Untitled.Gyroscope_x,Untitled.Gyroscope_y,Untitled.Gyroscope_z];

% data(:,3) = aRaw0(:,1);
% data(:, 2) = aRaw0(:,3);
% data(:, 1) = avRaw0(:,2);

%OBTAIN THE RAW VALUES FROM SENSORS AND PROCESS
% accel_x_raw = data(:, 3); %obtain force in y axis
% accel_z_raw = data(:, 2); %obtain force in z axis
% gyro_y_raw = data(:, 1); %obtain the gyro data
%scale the raw values
% accel_x = (accel_x_raw-512)*3.3/(1024*0.3336); %convert to g's
% accel_z = (accel_z_raw-512)*3.3/(1024*0.3336); %convert to g's
% gyro_y = (gyro_y_raw-421)*3.3/(1024*0.002);%convert to angular velocity (deg/s)
% gyro_y = gyro_y*-1; %have gyro rotate in right direction

accel_x = aRaw0(:,1);
accel_z = aRaw0(:,3);
gyro_y = avRaw0(:,2);

gyro_y = radtodeg(gyro_y);

accel_angle = atan2(accel_z, accel_x); %obtain the angle (rads) from the g's 
accel_angle = accel_angle.*180/pi; %convert to degrees
number_samples = length(gyro_y); %number of samples to be processed
x = 1:1:number_samples;
%IMPLEMENT THE KALMAN FILTER
%refer document D. Simon, “Kalman Filtering,” Embedded Systems Programming,
%vol. 14, no. 6, pp. 72-79, June 2001
%at http://academic.csuohio.edu/simond/publications.html
%for more information on the Kalman Filter algorithm
%and refer to
%http://www.electroiq.com/articles/sst/print/volume-54/issue-7/features/cover-article/solutions-for-mems-sensor-fusion.html
%for specific details about kalman filter with an acceleromter and
%gyroscope
Sz = 0.03*180/pi; %measurement noise
Sw = [0.001 0; 0 0.003].*180/pi; %process noise
P = eye(2);% initial estimation covariance
poshat = []; % estimated position array
bias = []; %store bias
xhat = [90 0]'; %state vector, inital position is 90 degrees
K_temp = ones(1,2);
K_log = []; %store K values
dt = .015; %15ms time step
a = [1 -dt; 
    0 1];
b = [dt 0]';
c = [1 0];

for t = 1 : 1: number_samples
    y = accel_angle(t); %measured value
    xhat = a*xhat + b*gyro_y(t); %state from process
Inn = y - c * xhat;
s = c * P * c' + Sz;
K = a * P * c' * inv(s);
xhat = xhat + K * Inn;
P = a * P * a' - a * P * c' * inv(s) * c * P * a' + Sw;
poshat = [poshat; xhat(1)];
bias = [bias; xhat(2)];
K_temp(1) = K(1); %retain the K_1 value
K_temp(2) = K(2); %retain the K_2 value
K_log = [K_log; K_temp];
end
figure, plot(x, poshat, 'k', x, accel_angle, 'r'); %plot the angles
title('Angular position Vs Time');
xlabel('sample');
ylabel('angular position (degrees)');
legend('filtered angle', 'accelerometer angle');
%figure, plot(bias); %plot the bias as determined by Kalman Filter

%OPTIMISE THE KALMAN FILTER
%Where the noise is not time varying the K terms tend towards a constant
figure, plot(x, K_log(:,1), 'k', x, K_log(:,2), 'r'); %plot K matrix
title('K Vs Time');
xlabel('sample');
ylabel('K array');
legend('K1', 'K2');
%since K tends to a constant it can be precomputed to simplyfy the
%implementation on a micro controller. Can reduce computational load and
%simplify the overall implementation. Using pre computed K matrix, the
%Kalman filter can be simplified to
%Inn = y - c * xhat
%xhat = xhat + K* Inn
% or 
%xhat = xhat + K *(y - c * xhat)
%expanding the matrix mulitplication
%angle(n+1) 
%  = angle(n) - dt*bias(n) + dt*gyro_reading + K_1 * [accelerometer_pos - angle(n)]
%bias(n+1) = bias(n) + K_2*[acceleromter_pos - angle(n+1)]
%where accelerometer_pos is the angular position obtained from the acceleromter (degrees)
%  angle is the filtered angle (degrees)
%  K_1 and K_2 are the elements of the K matrix in the Kalman Filter


%hard code K values based on the modal value (most often occuring value)
K_1 = mode(K_log(:,1));
K_2 = mode(K_log(:,2));
angle = 90; %initial position is 90 degrees
angle_log = [];
bias2 = 0;
bias2_log = [];
for t = 1 : 1 : number_samples
angle = angle - dt*bias2 + dt*gyro_y(t)+ K_1*(accel_angle(t) - angle);
bias2 = bias2 + K_2*(accel_angle(t) - angle);
angle_log = [angle_log; angle];
bias2_log = [bias2_log; bias2];
end
figure, plot(x, angle_log, 'k', x, accel_angle, 'r'); %plot angles with fixed K
title('Angular position Vs Time - Optimised Kalman Filter');
xlabel('sample');
ylabel('angular position (degrees)');
legend('filtered angle', 'accelerometer angle');