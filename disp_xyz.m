close all

[b1,a1] = butter(6,0.0001,'high');
[dataSize,~] = size(a);
aFilt = a;
for i = 1:3
    subplot(2,3,i)
    plot(t,a(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Raw Acceleration (m/sec^2)')
%     aFilt(:,i)=filtfilt(b1,a1,a(:,i));
    aFilt(:,i) = wden(a(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    thresh = max(abs(aFilt(1:300,i)))
    for j = 1:floor(dataSize/10)
        startAcc = ((j-1)*10) + 1;
        endAcc = j*10;
        if(max(abs(aFilt(startAcc:endAcc,i))) < thresh)
%             aFilt(startAcc:endAcc,i) = 0;
        end
    end
    startAcc = (j*10) + 1;
    endAcc = dataSize;
    if(max(abs(aFilt(startAcc:endAcc,i))) < thresh)
%         aFilt(startAcc:endAcc,i) = 0;
    end
    subplot(2,3,i+3)
    plot(t,aFilt(:,i),'r');
    xlabel('Time (sec)')
    ylabel('Filtered Acceleration (m/sec^2)')
end

time = t;

figure;
subplot(2,2,1)
plot(time,magNoG,'r')
xlabel('Time (sec)')
ylabel('Minus mean Acceleration (m/sec^2)')

order = 6; % 6th Order Filter
for i = 1:3
    [b1,a1] = butter(order,[0.016 0.05],'bandpass');
    magNoGf = aFilt(:,i);
    % plot(time,accxf,'r',time,accyf,'g',time,acczf,'b');

    subplot(2,2,3)
    if iterFc == 1
        plot(time,magNoGf);
        xlabel('Time (sec)')
        ylabel('Filtered Acceleration (m/sec^2)')
    end

    %% First Integration (Acceleration - Veloicty)

    velocitymagNoG=cumtrapz(time,magNoGf);

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

