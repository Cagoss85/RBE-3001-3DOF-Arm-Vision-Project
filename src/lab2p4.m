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

home = [0;0;0];
pp.servo_jp(home);
pause(1);

figure(1);
hold on;
grid on;
view(3);

title('Tip Positions');
xlabel('X  Position');
ylabel('Y Position');
zlabel('Z Position');

xArray = [];
yArray = [];
zArray = [];

for i = 1:10
    l = -40;
    h = 40;
    row1 = l + (h-l) .*rand(1,1); %generate random positions between -40 and 40 degrees
    row2 = l + (h-l) .*rand(1,1);
    row3 = l + (h-l) .*rand(1,1);
    randPos = [row1; row2; row3]; 
    
    pp.servo_jp(randPos); % go to random positions
    pause(1);
    pp.servo_jp(home);    % go to zero position
    pause(1);
    
    output{i} = pp.measured_cp();  %recording the zero position
    
    x = output{i}(1,4); %GET X val
    xArray = cat(1, xArray, x); %cat x array
    y = output{i}(2,4); %get y VAL
    yArray = cat(1, yArray, y); % cat y array
    z = output{i}(3,4); % get z value
    zArray = cat(1, zArray, z); %cat Z array
    
    %disp(x); 
    %disp(y);
    %disp(z);
    scatter3(x,y,z, 'filled', 'b'); % make 3d scatterplot with color filled
    pause(1);
end

scatter3(mean(xArray), mean(yArray), mean(zArray), '+r'); %plot the average position

homeData = cat(2,xArray,yArray, zArray);
writematrix(homeData, 'homeDataAaron.csv');

%disp(xArray);
%disp(yArray);
%disp(zArray);

goal = pp.goal_cp();
%disp(goal);\

disp(['xRMS is', num2str(sqrt(mean((xArray - goal(1,4)).^2)))]);
disp(['yRMS is', num2str(sqrt(mean((yArray - goal(2,4)).^2)))]);
disp(['zRMS is', num2str(sqrt(mean((zArray - goal(3,4)).^2)))]);

pp.shutdown();  %Shutdown robot












