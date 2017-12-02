close all;
% aFilt = a;
aFilt = [Untitled.Accelerometer_x,Untitled.Accelerometer_y,Untitled.Accelerometer_z];
av = [Untitled.Gyroscope_x,Untitled.Gyroscope_y,Untitled.Gyroscope_z];
t = Untitled.Time;
a = aFilt;
for i = 1:3
    subplot(2,3,i)
    plot(t,a(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Raw Acceleration (m/sec^2)')

%     aFilt(:,i) = wden(a(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    subplot(2,3,i+3)
    plot(t,aFilt(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Filtered Acceleration (m/sec^2)')
end

gRec = [mean(aFilt(:,1)),mean(aFilt(:,2)),mean(aFilt(:,3))];
gPol = gRec;
[gPol(1),gPol(2),gPol(3)] = cart2sph(gRec(1),gRec(2),gRec(3));

k = 0.98;
k1 = 1 - k;
time_diff = 0.01;

x = a(:,1);
y = a(:,2);
z = a(:,3);
radians_x = atan2(y(1),sqrt((x(1)*x(1))+(z(1)*z(1))));
degree_x = mod(radtodeg(radians_x), 360);
radians_y = atan2(x(1),sqrt((y(1)*y(1))+(z(1)*z(1))));
degree_y = -mod(radtodeg(radians_y), 360);
% radians_z = atan2(z,sqrt(x.^2+y.^2));
% degree_z = mod(radtodeg(radians_z), 360);

[len,~] = size(av);
[len2,~] = size(a);
len = min(len,len2);
last_x = zeros(len,1,'double');
last_y = zeros(len,1,'double');
gyro_scaled_x = zeros(len,1,'double');
gyro_scaled_y = zeros(len,1,'double');
rotation_x = zeros(len,1,'double');
rotation_y = zeros(len,1,'double');
last_x(1) = degree_x;
last_y(1) = degree_y;
gyro_offset_x = av(1,1);
gyro_offset_y = av(1,2);

gyro_total_x = last_x(1) - gyro_offset_x;
gyro_total_y = last_y(1) - gyro_offset_y;

for i = 2:len
    gyro_scaled_x(i) = av(i,1);
    gyro_scaled_y(i) = av(i,2);
    gyro_scaled_x(i) = gyro_scaled_x(i) - gyro_offset_x;
    gyro_scaled_y(i) = gyro_scaled_y(i) - gyro_offset_y;
    
    gyro_x_delta = (gyro_scaled_x(i) * time_diff);
    gyro_y_delta = (gyro_scaled_y(i) * time_diff);

    gyro_total_x = gyro_total_x + gyro_x_delta;
    gyro_total_y = gyro_total_y + gyro_y_delta;

    radians_x = atan2(y(i),sqrt((x(i)*x(i))+(z(i)*z(i))));
%     rotation_x(i) = mod(radtodeg(radians_x), 360);
    rotation_x(i) = radtodeg(radians_x);
    radians_y = atan2(x(i),sqrt((y(i)*y(i))+(z(i)*z(i))));
%     rotation_y(i) = -mod(radtodeg(radians_y), 360);
    rotation_y(i) = radtodeg(radians_y);

    last_x(i) = k * (last_x(i) + gyro_x_delta) + (k1 * rotation_x(i));
    last_y(i) = k * (last_y(i) + gyro_y_delta) + (k1 * rotation_y(i));
end

figure;
plot(t(1:len),x(1:len),'r',t(1:len),last_x(1:len),'g',t(1:len),rotation_x(1:len),'b');
figure,plot(t(1:len),y(1:len),'r',t(1:len),last_y(1:len),'g',t(1:len),rotation_y(1:len),'b');
% dist_z = sqrt((x.*x)+(y.*y));
% radians_z = atan(z ./ dist_z);
% degree_z = mod(radtodeg(radians_z), 360);
% 
% subplot(2,3,1)
% plot(t,x);
% xlabel('Time (sec)')
% ylabel('X (m/sec)')
% 
% subplot(2,3,2)
% plot(t,y);
% xlabel('Time (sec)')
% ylabel('Y (m/sec)')
% 
% subplot(2,3,3)
% plot(t,z);
% xlabel('Time (sec)')
% ylabel('Z (m/sec)')
% 
% subplot(2,3,4)
% plot(t,degree_x);
% xlabel('Time (sec)')
% ylabel('X Rotation (m/sec)')
% 
% subplot(2,3,5)
% plot(t,degree_y);
% xlabel('Time (sec)')
% ylabel('Y Rotation (m/sec)')
% 
% subplot(2,3,6)
% plot(t,degree_z);
% xlabel('Time (sec)')
% ylabel('Z Rotation (m/sec)')

