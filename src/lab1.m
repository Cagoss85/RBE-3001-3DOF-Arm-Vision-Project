clear
clear java
clear classes;

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
pp.measured_js(1,0);                %measure the position of the robot arm on startup

%**% Everything below here gets commented out while measuring robot postitions %**%

t=3000;   %interpolation time

%Send robot to zero position
zero = [0 0 0];
pp.servo_jp(zero);
 
pause(1); %needed to allow arm to reach 0 position
  
%Setting the position of the arm to the custom position
pos2 = [45 0 0];
pp.interpolate_jp(pos2,0);

%creating empty arrays for the stored points and time since last call
startPos = pp.measured_js(1,0);
positionValues = startPos(1,:);
timeValues = 0;

%arbitrarily set the previous position matrix 
elapsedTime = 0;
 
tic; %start the timer
%Set a loop that will iterate through k, keeping track of the time between changes, add each value to it's arraytimerVal = tic; %set timer 
while elapsedTime < t/1000
     currPos = pp.measured_js(1,0);                         % getting the current position of the arm
     positionValues = cat(1, positionValues, currPos(1,:)); % add the current positiion to the matrix
     elapsedTime = toc;                                     % stop the timer
     timeValues = cat(1, timeValues, elapsedTime);          % adding the time value to the matrix
end

allData = cat(2, timeValues, positionValues); %concantenating the time values to the position values
writematrix(allData, 'noTerpThree.csv'); %printing the storedarray to a CSV File
 
Array = csvread('allData.csv');

%Individual Plot creation
time = Array(:,1);              %sort out first column for time
jointOne = Array(:,2);          %Extract data for each joint
jointTwo = Array(:,3);
jointThree = Array(:,4);
 
%Subplot creation
figure(1)
subplot(3,1,1);
plot(time, jointOne);                   %plot joint pos vs time
title('Joint One Position vs Time');    %add title to plot
xlabel('Time (seconds)');               %add x axis label
ylabel('Joint One Position');           %add y axis label
axis([0 3 -5 45]);                      %specify window to compress noice / fix scale

subplot(3,1,2);
plot(time, jointTwo);
title('Joint Two Position vs Time');
xlabel('Time (seconds)');
ylabel('Joint Two Position');
axis([0 3 -5 45]);

subplot(3,1,3);
plot(time, jointThree);
title('Joint Three Position vs Time');
xlabel('Time (seconds)');
ylabel('Joint Three Position');
axis([0 3 -5 45]);

%Sort through timestamps to get intervals
deltaTime = time*1000;  %convert seconds to ms
prevTime = 0;           %initialize previous time
timeStep = [];          %initialize previous array

for k = 1:size(deltaTime)
    dur = deltaTime(k) - prevTime;
    timeStep = cat(1,dur, timeStep);
    prevTime = deltaTime(k);
end

%Histogram
figure(4)
histogram(timeStep);
title('Historgram of Time Steps');
xlabel('Time Steps (ms)');
ylabel('Number of Occurences');
xlim([0 5]);

pp.shutdown();  %Shutdown robot

toc