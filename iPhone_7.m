% clear 
close all
if(exist('m','var') == 1)
    discardlogs(m);
else
    connector on stevens123
    m = mobiledev;
end

disp('Connect your MATLAB mobile app to the IP address given above')
disp(' ')
disp('Enable sensors and start recording')
disp(' ')
input('Pess return key when finish recording data on mobile dev');

if(m.AccelerationSensorEnabled)
    m.AccelerationSensorEnabled = 0;
    [a,t] = accellog(m);
end
if(m.AngularVelocitySensorEnabled)
    m.AngularVelocitySensorEnabled = 0;
    [av, tav] = angvellog(m);
end

if(m.MagneticSensorEnabled)
    m.MagneticSensorEnabled = 0;
    [mag,tmag] = magfieldlog(m);
end
if(m.OrientationSensorEnabled)
    m.OrientationSensorEnabled = 0;
    [o, to] = orientlog(m);
end
if(m.PositionSensorEnabled)
    m.PositionSensorEnabled = 0;
    [lat, lon, tpos, spd] = poslog(m);
end

filename = ['ADSLab_', char(datetime('now','Format','MMdd_HHmm'))];
save(filename);