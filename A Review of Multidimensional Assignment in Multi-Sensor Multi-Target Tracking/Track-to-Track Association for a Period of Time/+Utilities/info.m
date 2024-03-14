function info(sensorNum, targetNum, distance)
    arguments
        sensorNum (1, 1) {mustBeInteger, mustBePositive}
        targetNum (1, 1) {mustBeInteger, mustBePositive}
        distance (1, 1) {mustBeReal, mustBePositive}
    end
    upperBorder = '--------------------------------------------------------------------';
    lowerBorder = upperBorder;
    borderLen = length(upperBorder);
    disp(upperBorder);
    str = "| " + string(sensorNum) + "-Sensor " + string(targetNum) + "-Target " + string(distance) + "-Distance Scenario Completed Successfully!";
    text = convertStringsToChars(str);
    for i = 1:(borderLen - length(text) - 1)
        str = append(str, " "); 
    end
    str = append(str, "|"); 
    disp(str);
    disp(lowerBorder);
end