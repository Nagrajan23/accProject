%% Get file from Raspberry Pi
% fileReadName = '20170817-1710.txt';
% fileReadName = [char(datetime('now','Format','yyyyMMdd-HHmm')-minutes(0)), '.txt'];

if(~exist(fileReadName,'file'))
    if(~exist('mypi','var'))
        mypi = raspi('155.246.44.43','pi','stevens123');
    end
    raspLocation = '/home/pi/mpuu9255/MPU9255/';
    for i = 0:10
        fileReadName = [char(datetime('now','Format','yyyyMMdd-HHmm')-minutes(i)), '.txt'];
        srcFile = [raspLocation,fileReadName];
        try
            getFile(mypi,srcFile);
            if(exist(fileReadName,'file'))
                break;
            end
        catch
            disp('Caught=');
            disp(fileReadName);
        end
    end

    if(exist('mypi','var'))
        clear mypi;
    end
end

%% Import data from text file.
% Script for importing data from the following text file:
%
%    C:\Users\Raj\Documents\MATLAB\Accelration\GitClone2\accProject\20170815-121531.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2017/08/15 14:16:15

%% Initialize variables.
filename = fileReadName;
delimiter = '\t';
startRow = 2;

%% Format for each line of text:
%   column1: datetimes (%{yyyy-MM-dd HH:mm:ss.SSSSSS}D)
%	column2: categorical (%C)
%   column3: categorical (%C)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%{yyyy-MM-dd HH:mm:ss.SSSSSS}D%C%C%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
Untitled = table(dataArray{1:end-1}, 'VariableNames', {'Time','Position_lat','Position_lng','Accelerometer_x','Accelerometer_y','Accelerometer_z','Gyroscope_x','Gyroscope_y','Gyroscope_z','Magnetometer_x','Magnetometer_y','Magnetometer_z'});

% For code requiring serial dates (datenum) instead of datetime, uncomment
% the following line(s) below to return the imported dates as datenum(s).

% Untitled.Time=datenum(Untitled.Time);

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;