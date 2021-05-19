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

%push
% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);  %create a robot object
%setting the speed Coeffiecient of the arm
speedCoefficient = 900;

p1 = [70 70 95];
p2 = [100 0 150];
p3 = [180 -50 60];
p = [p1;p2;p3;p1];

side = 1;
atVertex = false;
THRESH = 10;
isMoving = false;

tipData = [];
xData = [];
yData = [];
zData = [];
timeData = [];

disp("entering loop");
pp.servo_jp([45 29.6681 30.6638]);
inc = 1;
pause(1);

tic;
while side <= size(p,1)+1
    currAngle = pp.measured_js(1,0);
    currAngle = transpose(currAngle(1,:));

    qDotVector = pp.invVelKinematics(p(side,:), speedCoefficient);
    q1 = currAngle(1) + (qDotVector(1)/100);
    q2 = currAngle(2) + (qDotVector(2)/100);
    q3 = currAngle(3) + (qDotVector(3)/100);
    vect = [q1 q2 q3];
    
   pp.interpolate_jp(vect,10);
    
    currPos = pp.fk3001(currAngle,3);
    currPos = currPos((1:3),4);
    
    if isMoving == true &&  (floor(inc/40) == inc/40)
        xData = cat(1,xData,currPos(1));
        yData = cat(1,yData,currPos(2));
        zData = cat(1,zData,currPos(3));
        tipData = cat(2,xData,yData,zData);
        timeData = cat(1,timeData,toc);
    end
    
    xDist = abs(p(side,1)-currPos(1));
    yDist = abs(p(side,2)-currPos(2));
    zDist = abs(p(side,3)-currPos(3));
    
    if xDist < THRESH && yDist < THRESH && zDist < THRESH
        atVertex = true;
        isMoving = true;
    end
    
    if atVertex == true
        disp("Moving to the Next Side");
        side = side + 1;
        atVertex = false;
    end
    
    if side == 5
        break;
    end
    inc = inc+1;
end

disp("All Done!");

%Plot Position
figure(1);
subplot(3,1,1);
hold on;
plot(timeData(:,1),tipData(:,1));
plot(timeData(:,1),tipData(:,2));
plot(timeData(:,1),tipData(:,3));
grid on;
title('Position vs Time');
xlabel('Time (Seconds)');
ylabel('Position (mm)');
legend('X Position', 'Y Position', 'Z Position');

xDeriv = diff(tipData(:,1));
yDeriv = diff(tipData(:,2));
zDeriv = diff(tipData(:,3));

timeData1 = timeData;
timeData1(1,:) = [];

subplot(3,1,2);
hold on;
plot(timeData1,xDeriv);
plot(timeData1,yDeriv);
plot(timeData1,zDeriv);
grid on;
title('Velocity vs Time');
xlabel('Time (Seconds)');
ylabel('Velocity (mm/sec)');
legend('X Velocity', 'Y Velocity', 'Z Velocity');

xDeriv = diff(xDeriv);
yDeriv = diff(yDeriv);
zDeriv = diff(zDeriv);

timeData2 = timeData1;
timeData2(1,:) = [];

subplot(3,1,3);
hold on;
plot(timeData2,xDeriv);
plot(timeData2,yDeriv);
plot(timeData2,zDeriv);
grid on;
title('Acceleration vs Time');
xlabel('Time (Seconds)');
ylabel('Acceleration (mm/sec^2)');
legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');




pp.shutdown(); 












