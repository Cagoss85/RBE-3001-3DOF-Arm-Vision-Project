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


zero = pp.fk3001([0; 0; 0], 3);
new = [zero(1,4);
       zero(2,4);
       zero(3,4)]; 
pp.ik3001(new)


cal = pp.fk3001([-90; 86; 33.1], 3);
new = [cal(1,4);
       cal(2,4);
       cal(3,4)];
pp.ik3001(new)


arb1 = pp.fk3001([50; 30; -10], 3);
new = [arb1(1,4);
       arb1(2,4);
       arb1(3,4)];
pp.ik3001(new)


arb2 = pp.fk3001([-50; 50; -20], 3);
new = [arb2(1,4);
       arb2(2,4);
       arb2(3,4)];
pp.ik3001(new)



pp.shutdown();  %Shutdown robot
