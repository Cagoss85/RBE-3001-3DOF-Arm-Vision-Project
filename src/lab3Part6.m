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

p1 = [70 70 95];
p2 = [100 0 150];
p3 = [180 -50 60];
p = [p1;p2;p3;p1];

tp = Traj_Planner(p); %create trajectory planner object

%getting trajectory for the triangle sides
TS = cat(1, tp.linear_traj(p1, p2), tp.linear_traj(p2, p3), tp.linear_traj(p3, p1));


JA = pp.ik3001(transpose(TS(1,:)));

%making a cohesive list of all the joint angle values
for i = 2:size(TS)
    JA = cat(1, JA, pp.ik3001(transpose(TS(i,:))));
end

%creating initial arrays for storing measurements
pp.servo_jp(JA(1,:));
pause(3/15);
JAtest = pp.measured_js(1,0);
JAE = JAtest(1,:);

TStest = pp.fk3001(transpose(JAtest(1,:)), 3);
TSE = [TStest(1,4) TStest(2,4) TStest(3,4)]; 
time = 0;

%data collection
tic
for i = 2:size(JA)
    pp.servo_jp(JA(i,:));
    
    pause(3/15);
    time = cat(1, time, toc);
    
    JAtest = pp.measured_js(i,0);
    JAE = cat(1, JAE, JAtest(1,:));
    
    TStest = pp.fk3001(transpose(JAtest(1,:)), 3);
    TSE = cat(1, TSE, [TStest(1,4) TStest(2,4) TStest(3,4)]); 
    
    
end

%csvwrite('TSE.csv', TSE);
%csvwrite('JAE.csv', JAE);
%csvwrite('time.csv', time);


    
    

