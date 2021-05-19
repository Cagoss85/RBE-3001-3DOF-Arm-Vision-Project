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


x = true; 

% %run an infinite loop to constantly get and show the position of the arm
% while x 
%     clf;                                             %clearing the previous values 
%     
%     angleArray = pp.measured_js(1,0);                %getting the position of the arm in terms of angles
%     
%     pp.plot_arm(transpose(angleArray(1,:)));         %plotting the position of the arm
%     view(3);                                         %setting the plot to an isometric view
%     drawnow;                                         %updating the figure
% end
tic
moveAt = 0;
interval = 3; %Time between each arm movement
position = 1;
% positionArray = [45 45 -45];
positionArray = [-20 68 20];
% positionArray = [10 25 -10];
% positionArray = [-45 50 10];
% positionArray = [5 80 -45];
figure(1);

while x
    clf;  %clearing the previous values
    angleArray = pp.measured_js(1,0);                %getting the position of the arm in terms of angles
    pp.plot_arm(transpose(angleArray(1,:)));         %plotting the position of the arm
    view(3);                                         %setting the plot to an isometric view
    drawnow;
    
    if toc > moveAt && position <= size(positionArray,1)
        pp.servo_jp(positionArray);
        moveAt = moveAt + interval;
        position = position + 1;
    end
    
    if toc > moveAt && position > size(positionArray,1) 
        clf;                                        %clearing the previous values
        angleArray = pp.measured_js(1,0);           %getting the position of the arm in terms of angles
        pp.plot_arm(transpose(angleArray(1,:)));    %plotting the position of the arm
        view(3);                                    %setting the plot to an isometric view
        drawnow;
        disp(pp.measured_cp());
        break;
    end
                                           %updating the figure
end

pp.shutdown();  %Shutdown robot


