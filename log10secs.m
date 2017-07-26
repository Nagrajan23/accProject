% clear
% m = mobiledev;
% 
% m.AccelerationSensorEnabled = 1;
% m.AngularVelocitySensorEnabled = 1;
% m.MagneticSensorEnabled = 1;
% m.OrientationSensorEnabled = 1;
% m.PositionSensorEnabled = 1;
% m.SampleRate = 100;
% 
% m.Logging = 1;
% pause(200);
% m.Logging = 0;

if(m.AccelerationSensorEnabled)
    m.AccelerationSensorEnabled = 0;
    [a,t] = accellog(m);
end
if(m.AngularVelocitySensorEnabled)
    m.AngularVelocitySensorEnabled = 0;
    [ang,tang] = angvellog(m);
end
if(m.MagneticSensorEnabled)
    m.MagneticSensorEnabled = 0;
    [mag,tmag] = magfieldlog(m);
end
if(m.OrientationSensorEnabled)
    m.OrientationSensorEnabled = 0;
    [ori,tori] = orientlog(m);
end
if(m.PositionSensorEnabled)
    m.PositionSensorEnabled = 0;
    [pos,tpos] = poslog(m);
end

% clear m;