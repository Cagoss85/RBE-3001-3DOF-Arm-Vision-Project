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

val = [0 90 -90];
%val = [-90 86.1 33.7];
pp.interpolate_jp(val,500);
pause(.125);
disp("measuring");
status = pp.measured_js(1,1); %Pos, Vel
currAngle = transpose(status(1,:))
currVel = transpose(status(2,:))

pp.fdk3001(currAngle, currVel)

pp.shutdown();  %Shutdown robot

