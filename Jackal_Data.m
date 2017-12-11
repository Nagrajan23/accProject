close all
% clear

tic
bagFilename = 'imu_1311_1.bag'; %Filename to be imported
if(~exist(bagFilename,'file'))
    if(~exist('rosDevice1','var'))
        %Jackal Robot IP, usernmae and password
        rosDevice = rosdevice('IP_address','username','password');
    end
    getFile(rosDevice,['~/', bagFilename]);
    toc
    disp('File Received');
end
rosBagAll = rosbag(bagFilename);
disp('File Imported');
% For getting IMU data
imuBag = select(rosBagAll,'Topic','/imu/data');
imurawBag = select(rosBagAll,'Topic','/imu/data_raw');
cmdvelBag = select(rosBagAll,'Topic','/jackal_velocity_controller/cmd_vel');
odomfiltBag = select(rosBagAll,'Topic','/odometry/filtered');
toc
disp('Topics are selected');
imuData = readMessages(imuBag);
imurawData = readMessages(imurawBag);
toc
disp('Read IMU Data');
odomfiltData = readMessages(odomfiltBag);
toc
disp('Read odom Messages');