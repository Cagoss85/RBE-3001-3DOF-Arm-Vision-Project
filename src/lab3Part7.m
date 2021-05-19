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

posData = [];   %Set empty matrix for position data collection

%Set Positions
p1 = [70 70 95];
p2 = [100 0 150];
p3 = [180 -50 60];
p = [p1;p2;p3;p1];
ang = [45 29.6681 30.6638];
pp.servo_jp(ang);
pause(1);
tic;

%Loop commented out because it isnt needed
% for i=1:size(p)
%     q = transpose(p(i,:));
%     ik = pp.ik3001(q);
%     ang = [ik(1) ik(2) ik(3)]
%     pp.servo_jp(ang);
%     pause(1);
%     currPos = pp.measured_js(1,0);
%     currPos = transpose(currPos(1,:));
%     fk = pp.fk3001(currPos,3);
%     fk = transpose(fk((1:3),4));
%     dataRow = cat(2, fk, ang, toc);
%     posData = cat(1,posData, dataRow);
%     
% end

tp = Traj_Planner(p); %create trajectory planner object
STEP = 20;
interval = 0:.1:3;

%Based on desired positions, create the trajectory eqns
for i=1:size(p)-1
syms t;
x{i} = tp.quintic_traj(0, 3,0, 0, p(i,1),p(i+1,1));
y{i} = tp.quintic_traj(0, 3,0, 0, p(i,2),p(i+1,2));
z{i} = tp.quintic_traj(0, 3,0, 0, p(i,3),p(i+1,3));
qx{i} = x{i}(1) + x{i}(2)*t +x{i}(3)*t^2+x{i}(4)*t^3+x{i}(5)*t^4+x{i}(6)*t^5;
qy{i} = y{i}(1) + y{i}(2)*t +y{i}(3)*t^2+y{i}(4)*t^3+y{i}(5)*t^4+y{i}(6)*t^5;
qz{i} = z{i}(1) + z{i}(2)*t +z{i}(3)*t^2+z{i}(4)*t^3+z{i}(5)*t^4+z{i}(6)*t^5;
xData{i} = subs(qx{i},t,interval);
yData{i} = subs(qy{i},t,interval);
zData{i} = subs(qz{i},t,interval);
end

%Position values for robot
side1 = cat(1,xData{1},yData{1},zData{1});
side2 = cat(1,xData{2},yData{2},zData{2});
side3 = cat(1,xData{3},yData{3},zData{3});


%Move the robot to the calculated positions for each triangle edge
cubicData = [];
tic;
for w = 1:size(side1,2)
    ja = pp.ik3001(side1(:,w));
    pp.servo_jp(ja);
     currPos = pp.measured_js(1,0);
    currPos = transpose(currPos(1,:));
    fk = pp.fk3001(currPos,3);
    fk = transpose(fk((1:3),4));
    dataRow = cat(2, fk, toc);
    cubicData = cat(1,cubicData, dataRow);
end

for w = 1:size(side2,2)
    ja = pp.ik3001(side2(:,w));
    pp.servo_jp(ja);
    currPos = pp.measured_js(1,0);
    currPos = transpose(currPos(1,:));
    fk = pp.fk3001(currPos,3);
    fk = transpose(fk((1:3),4));
    dataRow = cat(2, fk, toc);
    cubicData = cat(1,cubicData, dataRow);
end

for w = 1:size(side3,2)
    ja = pp.ik3001(side3(:,w));
    pp.servo_jp(ja);
    currPos = pp.measured_js(1,0);
    currPos = transpose(currPos(1,:));
    fk = pp.fk3001(currPos,3);
    fk = transpose(fk((1:3),4));
    dataRow = cat(2, fk, toc);
    cubicData = cat(1,cubicData, dataRow);
end

%plot the position data
figure(1);
subplot(3,1,1);
title("X, Y, and Z Position vs. Time");
xlabel("Time (Seconds)");
ylabel("Position (mm)");
hold on;
grid on;
plot(cubicData(:,4),cubicData(:,1));
plot(cubicData(:,4),cubicData(:,2));
plot(cubicData(:,4),cubicData(:,3));
legend('X Position', 'Y Position', 'Z Position');

%Plot the velocity Data
subplot(3,1,2);
title("X, Y, and Z Velocity vs. Time");
xlabel("Time (Seconds)");
ylabel("Velocity (mm/sec)");
hold on;
grid on;
timediff = cubicData(:,4);  %Required to make vectors same size
timediff(1,:) = [];
plot(timediff,diff(cubicData(:,1)));    %Plot derivative of position matrix
plot(timediff,diff(cubicData(:,2)));
plot(timediff,diff(cubicData(:,3)));
legend('X Velocity', 'Y Velocity', 'Z Velocity');

%Plot the acceleration data
subplot(3,1,3);
title("X, Y, and Z Acceleration vs. Time");
xlabel("Time (Seconds)");
ylabel("Acceleration (mm/sec^2)");
hold on;
grid on;
timediff(1,:) = []; %Required again to make vectors same size
plot(timediff,diff(cubicData(:,1),2));  %Plot derivative of velocity graph
plot(timediff,diff(cubicData(:,2),2));
plot(timediff,diff(cubicData(:,3),2));
legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');

pp.shutdown();

