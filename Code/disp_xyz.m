close all

[b1,a1] = butter(6,0.0001,'high');
[dataSize,~] = size(a);
aFilt = a;
for i = 1:3
%     subplot(2,3,i)
%     plot(t,a(:,i),'r');
%     xlabel('Time (sec)')
%     ylabel('Raw Acceleration (m/sec^2)')
%     aFilt(:,i)=filtfilt(b1,a1,a(:,i));
    aFilt(:,i) = wden(a(:,i),'modwtsqtwolog','s','mln',8,'sym4');
    
    gVal = mean(aFilt(1:300,i));
    aFilt(:,i) = aFilt(:,i) - gVal;
    
    thresh = max(abs(aFilt(1:300,i)))
    for j = 1:floor(dataSize/10)
        startAcc = ((j-1)*10) + 1;
        endAcc = j*10;
        if(max(abs(aFilt(startAcc:endAcc,i))) < thresh)
            aFilt(startAcc:endAcc,i) = 0;
        end
    end
    startAcc = (j*10) + 1;
    endAcc = dataSize;
    if(max(abs(aFilt(startAcc:endAcc,i))) < thresh)
        aFilt(startAcc:endAcc,i) = 0;
    end
%     subplot(2,3,i+3)
%     plot(t,aFilt(:,i),'r');
%     xlabel('Time (sec)')
%     ylabel('Filtered Acceleration (m/sec^2)')
    
    magNoG = aFilt(:,i);
    magNoGf = magNoG;
    time = t;

    figure;
    subplot(2,2,1)
    plot(time,magNoG,'r')
    xlabel('Time (sec)')
    ylabel('Minus mean Acceleration (m/sec^2)')
    
    %% First Integration (Acceleration - Veloicty)
    velocitymagNoG=cumtrapz(time,magNoGf);
    
    %% Filter Veloicty Signals
    velmagNoGf = velocitymagNoG;
    [velSize,~] = size(velocitymagNoG);
    for i = 1:floor(velSize/10)
        startVel = ((i-1)*10) + 1;
        endVel = i*10;
        if(max(abs(velmagNoGf(startVel:endVel))) < 0.1)
%             velmagNoGf(startVel:endVel) = 0;
        end
    end
    startVel = (i*10) + 1;
    endVel = velSize;
    if(max(abs(velmagNoGf(startVel:endVel))) < 0.1)
%         velmagNoGf(startVel:endVel) = 0;
    end
    
    subplot(2,2,2)
    plot(time,velmagNoGf);
    xlabel('Time (sec)')
    ylabel('Velocity (m/sec)')
    
    %% Second Integration (Velocity - Displacement)
    DisplacementmagNoG=cumtrapz(time, velmagNoGf);
    
    subplot(2,2,4)
    plot(time,DisplacementmagNoG);
    xlabel('Time (sec)')
    ylabel('Displacement (m)')
end