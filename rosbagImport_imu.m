clear
tic
bagFilename = 'imu_1710_2.bag';
if(~exist(bagFilename,'file'))
    if(~exist('rosDevice1','var'))
        rosDevice = rosdevice('10.10.10.101','administrator','clearpath');
%         rosDevice1 = rosdevice('192.168.43.79','administrator','clearpath');
    end
    getFile(rosDevice,['~/', bagFilename]);
    toc
    disp('File Received');
end

rosBagAll = rosbag(bagFilename);
disp('File Imported');

% For getting velodyne data
% veloBag = select(rosBagAll,'Topic','/velodyne_points_2Hz');
% odomBag = select(rosBagAll,'Topic','odom_2Hz');

% For getting IMU data
imuBag = select(rosBagAll,'Topic','/imu/data');
imurawBag = select(rosBagAll,'Topic','/imu/data_raw');
cmdvelBag = select(rosBagAll,'Topic','/jackal_velocity_controller/cmd_vel');
odomfiltBag = select(rosBagAll,'Topic','/odometry/filtered');

toc
disp('Topics are selected');
% For reading messages
% odomData = readMessages(odomBag);

imuData = readMessages(imuBag);
imurawData = readMessages(imurawBag);
toc
disp('Read IMU Data');
% cmdvelData = readMessages(cmdvelBag);
% disp('Read cmd');
odomfiltData = readMessages(odomfiltBag);
toc
disp('Read odom Messages');


% topicsTable = veloBag.AvailableTopics;
% numMessages = topicsTable{1,1};
% if(numMessages <= 200)
%     lidarData = readMessages(veloBag);
%     disp('Data Extracted');
%     lidarData = repmat(readMessages(veloBag,1),numMessages,1);
%     for i = 1:ceil(numMessages/200)
%         fromVal = (((i-1)*200) + 1);
%         toVal = i*200;
%         if(toVal > numMessages)
%             toVal = numMessages;
%         end
%         lidarData(fromVal:toVal) = readMessages(veloBag,fromVal:toVal);
%     end
% end