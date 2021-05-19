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

posData = [];

p1 = [70 70 95];
p2 = [100 0 150];
p3 = [180 -50 60];
p = [p1;p2;p3;p1];
ang = [45 29.6681 30.6638];
ang1 = [45 29.6681 30.6638];
pp.servo_jp(ang);
prevTime = 0;
interval = 0;
pos = 1;
tic;
figure(1);
grid on;
while pos <= size(p,1)
    clf;
    pp.plot_arm(transpose(ang));
    view(3);                                         %setting the plot to an isometric view
    drawnow;
    
    if toc > interval && pos <= 4
        q = transpose(p(pos,:));
        ik = pp.ik3001(q);
        ang = [ik(1) ik(2) ik(3)]
        interval = interval + 2;
        pos = pos + 1;
        pp.interpolate_jp(ang,1000);
        %pause(1);
    end
    
    currPos = pp.measured_js(1,0);
    currPos = transpose(currPos(1,:));
    fk = pp.fk3001(currPos,3);
    fk = transpose(fk((1:3),4));
    dataRow = cat(2, fk, ang, toc);
    posData = cat(1,posData, dataRow);
    
    if toc > interval && pos == 5
        pause(1);
        clf;
        pp.plot_arm(transpose(ang1));
        view(3);                                         %setting the plot to an isometric view
        drawnow;
        break;
    end
end

figure(2);
title("X, Y, and Z Position vs. Time");
xlabel("Time (Seconds)");
ylabel("Position (mm)");
hold on;
plot(posData(:,7),posData(:,1));
plot(posData(:,7),posData(:,2));
plot(posData(:,7),posData(:,3));
legend('X Position', 'Y Position', 'Z Position');

figure(3);
plot3(posData(:,1),posData(:,2),posData(:,3));
grid on;
view(3);
title("3D Plot of Arm Motion");
xlabel("X Position (mm)");
ylabel("Y Position (mm)");
zlabel("Z Position (mm)");

figure(4);
grid on;
title("Joint Angle vs. Time");
hold on;
plot(posData(:,7),posData(:,4));
plot(posData(:,7),posData(:,5));
plot(posData(:,7),posData(:,6));
xlabel("Time (Seconds)");
ylabel("Joint Angle (deg)");
legend('q1', 'q2', 'q3');

posData

pp.shutdown();


