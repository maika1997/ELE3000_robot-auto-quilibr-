function readSerial(src, ~)
    % Read the ASCII data from the serialport object.
    newData = readline(src);

    % Check if input is data
    if strncmpi(newData,"%",1)

        splitData = split(newData, ',');


        temps = str2double(eraseBetween(splitData(1), 1, 1));
        angle = str2double(splitData(2));
%        speed_motorL = str2double(splitData(3));
%        speed_motorR = str2double(splitData(4));
        pwm = str2double(splitData(3));
        kpAction = str2double(splitData(4));
        velocity = str2double(splitData(5));
        kdAction = str2double(splitData(6));
        
        showtime = temps - src.UserData.StartTime
        
        % Check if initial time is set
        if src.UserData.StartTime == -1
            src.UserData.StartTime = temps;
        end
        % Convert the string data to numeric type and save it in the UserData
        % property of the serialport object.
        src.UserData.Temps(end+1) = temps;
        src.UserData.Angle(end+1) = angle;
%        src.UserData.SpeedMotorL(end+1) = speed_motorL;
%        src.UserData.SpeedMotorR(end+1) = speed_motorR;
        src.UserData.pwm(end+1) = pwm;
        src.UserData.KpAction(end+1) = kpAction;
        src.UserData.velocity(end+1) = velocity;
        src.UserData.KdAction(end+1) = kdAction;

        % Update the Count value of the serialport object.
        src.UserData.Count = src.UserData.Count + 1;

        % If 1001 data points have been collected from the Arduino, switch off the
        % callbacks and plot the data.
        if temps - src.UserData.StartTime > src.UserData.FinalTime
            src.UserData.Temps = src.UserData.Temps - src.UserData.Temps(1);
            configureCallback(src, "off");    
        end
    end
end