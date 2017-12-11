% clear all
close all

% File_name in Raspberry Pi 3 for logged data of MPU9255
filename = '20171113-1659.txt';

if(~exist(filename,'file'))
    if(~exist('mypi','var'))
        %Raspberry Pi IP, usernmae and password
        mypi = raspi('IP_address','username','password'); 
    end
    raspLocation = '/home/pi/mpuu9255/MPU9255/';
    srcFile = [raspLocation,filename];
    % For importing file from Raspberry Pi
    getFile(mypi,srcFile);

    if(exist('mypi','var'))
        clear mypi;
    end
end
%% Initialize variables.
delimiter = '\t';
startRow = 2;
%% Read columns of data as text:
formatSpec = '%s%*s%*s%s%s%s%s%s%s%[^\n\r]';
%% Open the text file.
fileID = fopen(filename,'r');
%% Read columns of data according to the format.
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