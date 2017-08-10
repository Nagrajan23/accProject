% x = a(:,1);
% y = a(:,2);
% z = a(:,3);
% dist = a(:,3);
% radians = a(:,3);
% 
% dist_x = sqrt((y.*y)+(z.*z));
% radians_x = atan(x ./ dist_x);
% degree_x = mod(radtodeg(radians_x), 360);
% 
% dist_y = sqrt((x.*x)+(z.*z));
% radians_y = atan(y ./ dist_y);
% degree_y = mod(radtodeg(radians_y), 360);
% 
close all;
aFilt = a;
for i = 1:3
    subplot(2,3,i)
    plot(t,a(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Raw Acceleration (m/sec^2)')

    aFilt(:,i) = wden(a(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    subplot(2,3,i+3)
    plot(t,aFilt(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Filtered Acceleration (m/sec^2)')
end

gRec = [mean(aFilt(:,1)),mean(aFilt(:,2)),mean(aFilt(:,3))];
gPol = cart2sph(gRec(1),gRec(2),gRec(3));