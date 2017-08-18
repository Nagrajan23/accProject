initial_estimate = 68;
estErr = 2;
mErr = 4;
inputVals = [75 71 70 74];
estPrev = initial_estimate;
[~,len] = size(inputVals);

for i = 1:len
    i
    kalmanGain = estErr / (estErr + mErr)
    estCur = estPrev + kalmanGain * (inputVals(i) - estPrev);
    estErr = (1 - kalmanGain) * estErr;
    estPrev = estCur;
    estCur
    estErr
end