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
pp.servo_jp([0 30 30]);
%deliberatly send robot to a singularity (this case, straight up)
figure(1);
view(3);
increment = 0;
atSingularity = false;
moving = false;
tipData = [];
xData = [];
yData = [];
zData = [];
detData = [];
singData = [];
timeData = [];

while true
    if atSingularity == true
        pp.shutdown();
        break;
    end
    if increment == 25
        pp.interpolate_jp([0 0 -90],1000);
        moving = true;
        tic;
    end
    pos = pp.measured_js(1,0);
    pos = transpose(pos(1,:));
    taskPos = pp.fk3001(pos,3);
   
    clf;
    view(3);
    pp.plot_arm(pos);
    drawnow();
    j = pp.jacob3001(pos);
    jMatrix = j((1:3),:);
    singCheck = det(jMatrix)/10000;
    
    if moving == true
        xData = cat(1,xData,taskPos(1,4));
        yData = cat(1,yData,taskPos(2,4));
        zData = cat(1,zData,taskPos(3,4));
        tipData = cat(2,xData,yData,zData);
        
        singData = cat(1,singData,singCheck);
        timeData = cat(1,timeData,toc);
        detData = cat(2,singData,timeData);
    end
    
    if(abs(singCheck) <= 8)
        disp("This point is near a singularity!");
        atSingularity = true;
    end
    increment = increment + 1;
end

%plot 3d Path
figure(2);
subplot(2,1,1);
view(3);
plot3(tipData(:,1),tipData(:,2),tipData(:,3));
grid on;
title("Tip Position Movement");
axis equal;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
xlim([-20 250]);
ylim([-140 180]);
zlim([0 300]);

%Plot Determinant
subplot(2,1,2);
plot(detData(:,2),detData(:,1));
grid on;
title('Determinant vs Time');
xlabel('Time (Seconds)');
ylabel('Jacobian Determinant (/10000)');
pp.interpolate_jp([0 0 0],1000);

