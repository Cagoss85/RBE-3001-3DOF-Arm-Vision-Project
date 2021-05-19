clear
clear java
clear classes;
clear all;
clc;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);  %create a robot object


%it starts and ends in the same position
tipPositionArray = [[0   37.895   32.864];
                    [0   0       -42.648];
                    [0   30.498  -42.069];
                    [0   37.895   32.864]];

%initialize arrays in starting position to store data in to convert to csv file
angleData = tipPositionArray(1,:);   
tipDataX = 94.3759;    
tipDataY = 0.0;     
tipDataZ = 79.4997;    
timeData = 0;
             
interval = 3;                            %starting time to allow arm to go from calibration position to position 1 
position = 1;                            %initialization for positionArray index

elapsedTime = 0;                         %initialization for recording data
currentAngles = 0;                       
currentTipPosition = 0; 
x = 1;

pp.interpolate_jp(tipPositionArray(position,:), 3)   %move the arm to position 1 
pause(3);
tic                                               %start timer

while x 
    
    if toc > interval && position < 4                                     %allows for incrementation of position and time threshold
        interval = interval + 3;
        position = position + 1;
        pp.interpolate_jp(tipPositionArray(position,:), 3)                   %send the arm to the next position 
    end
    
    elapsedTime = toc;                                           
    timeData = cat(1, timeData, elapsedTime);                    %record time data
    
    
    currentAngles = pp.measured_js(1,0);                         %getting the position of the arm in terms of angles
    angleData = cat(1, angleData, currentAngles(1,:));           %record angle data
    
    currentTipPosition = pp.fk3001(transpose(currentAngles(1,:)), 3);       
    tipDataX = cat(1, tipDataX, currentTipPosition(1,4));        %record tip position data
    tipDataY = cat(1, tipDataY, currentTipPosition(2,4));
    tipDataZ = cat(1, tipDataZ, currentTipPosition(3,4));
    
    if toc > interval && position == 4
        break
    end
    
end

%convert joint data matrices into csv file
angleData = cat(2, timeData, angleData);
writematrix(angleData, 'angleData.csv');

%convert tip position matrices into csv file
XZData = cat(2, timeData, tipDataX, tipDataY, tipDataZ);
writematrix(XZData, 'XYZData.csv');



pp.shutdown();






