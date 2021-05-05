clc;
clear all;

% Parameters
portNumber = "COM5";
captureTime = 15;

% Setup Serial Comm
serialportlist("available");
teensyObj = serialport(portNumber,9600);
teensyObj.UserData = struct("Temps",[], ...
                            "Angle", [], ...
                            "SpeedMotorL", [], ...
                            "SpeedMotorR", [], ... 
                            "pwm", [], ...
                            "KpAction", [], ... % Kp*error
                            "velocity", [], ... % angular velocity
                            "KdAction", [], ... % Kd*angleVel
                            "Count", 1, ... % Total Number of data
                            "StartTime", -1, ...
                            "FinalTime", captureTime); % Final time
configureTerminator(teensyObj,"CR/LF");

runScript = true;

while runScript
    userInput = input("\nCommand to execute? ", 's');
    userInput = split(userInput, " ");
    command = userInput{1};
    
    switch command
        case "data"
            collectData(teensyObj, captureTime)
        
        case "Kp"
            if length(userInput) == 2
                setGains(teensyObj, "Kp", str2double(userInput{2}));
            end
            
        case "Ki"
            if length(userInput) == 2
                setGains(teensyObj, "Ki", str2double(userInput{2}));
            end
            
        case "Kd"
            if length(userInput) == 2
                setGains(teensyObj, "Kd", str2double(userInput{2}));
            end
            
        case "plot"
            plotData(teensyObj.UserData)
        
        case "save"
            if length(userInput) == 2
                saveData(teensyObj.UserData, userInput{2})
            else
                saveData(teensyObj.UserData, "")
            end
            
        case "load"
            if length(userInput) == 2
                teensyObj.UserData = loadData(userInput{2});
            else
                fprintf("Provide a file name");
            end
            
        case "help"
            help()
            
        case "exit"
            runScript = false;
            
        otherwise
            fprintf("Invalid input\n")
    end
end

% Read teensy data from serial port
function collectData(serialObj, timeToRun)
    fprintf("Collecting data...\n")
    serialObj.UserData = struct("Temps",[], ...
                                "Angle", [], ...
                                "SpeedMotorL", [], ...
                                "SpeedMotorR", [], ... 
                                "pwm", [], ...
                                "KpAction", [], ... % Kp*error
                                "velocity", [], ... % angular velocity
                                "KdAction", [], ... % Kd*angleVel
                                "Count", 1, ... % Total Number of data
                                "StartTime", -1, ...
                                "FinalTime", timeToRun); % Final time
    % Read Serial Data
    flush(serialObj);
    
    % Start controller
    configureCallback(serialObj,"terminator",@readSerial);
    writeline(serialObj,"#Start"); % Send Start command
    waitfor(serialObj, "BytesAvailableFcnMode", "off") % Wait for data to be taken
    writeline(serialObj,"#Stop"); % Send Stop command
%     readline(serialObj)

    serialObj.UserData.Temps = serialObj.UserData.Temps - serialObj.UserData.Temps(1);
end

% Set Gains
function setGains(serialObj, gainIdx, gainVal)
    flush(serialObj);
    switch gainIdx
        case "Kp"
            fprintf("Setting Kp to: %.2f \n", gainVal)
            writeline(serialObj,"#Kp " + gainVal); % Send Start command
            fprintf(readline(serialObj))
            fprintf(readline(serialObj))
            
        case "Ki"
            fprintf("Setting Ki to: %.2f \n", gainVal)
            writeline(serialObj,"#Ki " + gainVal); % Send Start command
            fprintf(readline(serialObj))
            fprintf(readline(serialObj))
            
        case "Kd"
            fprintf("Setting Kd to: %.2f \n", gainVal)
            writeline(serialObj,"#Kd " + gainVal); % Send Start command
            fprintf(readline(serialObj));
            fprintf(readline(serialObj));
    end
end

% Plot
function plotData(data)
    fprintf("Ploting data\n")
    figure
    ax1 = subplot(2, 1, 1);
    hold on
        plot(data.Temps, data.Angle, 'b');
        plot(data.Temps, data.pwm, 'k');
        legend("angle [deg]");
        xlabel("Time [sec]")
        ylabel("Angle [deg], PWM")
        legend("Angle[deg]","PWM");
        grid on
    hold off

    ax2 = subplot(2, 1, 2);
    hold on
        plot(data.Temps, data.pwm, 'k');
        plot(data.Temps, data.KpAction, 'b');
%         plot(data.Temps, data.KiAction, 'c');
        plot(data.Temps, data.KdAction, 'r');
        plot(data.Temps, data.velocity, 'g');
        legend("PWM","Kp", "Kd");
        xlabel("Time [sec]")
        grid on
    hold off
    linkaxes([ax1, ax2], 'x');
end

% Save Data
function saveData(data, saveName)
    saveFile = "Data/" + saveName + ".mat";
    fprintf("Saving to: " + saveFile);
    save(saveFile, 'data');
end

% Load Data
function data = loadData(fileName)
    filePath = "Data/" + fileName + ".mat";
    data = load(filePath).data;
end

% Help
function help()
   fprintf("HELP\n")
   fprintf("Possible commands:\n")
   fprintf("\t data: Read data from teensy\n")
   fprintf("\n")
end
