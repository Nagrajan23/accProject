clear
% Get file from Raspberry Pi
% fileReadName = '20170831-1211.txt';
fileReadName = [char(datetime('now','Format','yyyyMMdd-HHmm')-minutes(0)), '.txt'];

if(~exist(fileReadName,'file'))
    if(~exist('mypi','var'))
<<<<<<< HEAD
        mypi = raspi('155.246.44.54','pi','stevens123');
=======
        mypi = raspi('155.246.169.52','pi','stevens123');
>>>>>>> 1f1047a873cd4b75af2e1a98f2175e6d38e2f28e
%         openShell(mypi)
    end
    raspLocation = '/home/pi/mpuu9255/MPU9255/';
    for i = 0:10
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
        fileReadName = [char(datetime('now','Format','yyyyMMdd-HHmm')-minutes(i)), '.txt'];
    end

    if(exist('mypi','var'))
        clear mypi;
    end
end

%% Import data from text file.
% Script for importing data from the following text file:
%
%    C:\Users\Raj\Documents\MATLAB\Accelration\GitClone2\accProject\20170818-1705.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2017/08/18 17:37:01

%% Initialize variables.
filename = fileReadName;
delimiter = '\t';
startRow = 2;

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%*s%*s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[2,3,4,5,6,7]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end

% Convert the contents of columns with dates to MATLAB datetimes using the
% specified date format.
try
    dates{1} = datetime(dataArray{1}, 'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSS', 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
catch
    try
        % Handle dates surrounded by quotes
        dataArray{1} = cellfun(@(x) x(2:end-1), dataArray{1}, 'UniformOutput', false);
        dates{1} = datetime(dataArray{1}, 'Format', 'yyyy-MM-dd HH:mm:ss.SSSSSS', 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSS');
    catch
        dates{1} = repmat(datetime([NaN NaN NaN]), size(dataArray{1}));
    end
end

anyBlankDates = dataArray{1} == '';
anyInvalidDates = isnan(dates{1}.Hour) - anyBlankDates;
dates = dates(:,1);

%% Split data into numeric and string columns.
rawNumericColumns = raw(:, [2,3,4,5,6,7]);

%% Exclude rows with non-numeric cells
I = ~all(cellfun(@(x) (isnumeric(x) || islogical(x)) && ~isnan(x),rawNumericColumns),2);  %Find rows with non-numeric cells
K = I | anyInvalidDates | anyBlankDates;
NumOfRowsExcluded = sum(K)
[~,rowsExcludedIndex] = max(K)
dates = cellfun(@(x) x(~K,:), dates, 'UniformOutput', false);
rawNumericColumns(K,:) = [];

%% Create output variable
Untitled = table;
Untitled.Time = dates{:, 1};
Untitled.Accelerometer_x = cell2mat(rawNumericColumns(:, 1));
Untitled.Accelerometer_y = cell2mat(rawNumericColumns(:, 2));
Untitled.Accelerometer_z = cell2mat(rawNumericColumns(:, 3));
Untitled.Gyroscope_x = cell2mat(rawNumericColumns(:, 4));
Untitled.Gyroscope_y = cell2mat(rawNumericColumns(:, 5));
Untitled.Gyroscope_z = cell2mat(rawNumericColumns(:, 6));

% For code requiring serial dates (datenum) instead of datetime, uncomment
% the following line(s) below to return the imported dates as datenum(s).

% Untitled.Time=datenum(Untitled.Time);

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp dates blankDates anyBlankDates invalidDates anyInvalidDates rawNumericColumns I J K;