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

%                 %measure the position of the robot arm on startup
% 
% syms theta d a alpha;
% symsVariables1 = [theta, d, a, alpha];
% % 
% syms L0 L1 L2 L3 q1 q2 q3 a1;
% symsVariables2 = [L0, L1, L2, L3, q1, q2, q3, a1];
% % 
% link1 = [q1,    L0+L1,  0,      a1];
% link2 = [q2-(pi/2), 0,      L2,     0]; 
% link3 = [q3+(pi/2), 0,      L3,     0];
% links = [link1;link2;link3];
%disp(links);
% 
% %**% Everything below here gets commented out while measuring robot postitions %**%
% %disp(pp.dh2mat(symsVariables1));
% 
% trans = pp.dh2fk(links);
% disp(trans{1});
% disp(trans{2});
% disp(trans{3});
% 
% pos= [0; 0; 0];
% pp.fk3001(pos);
% disp(pp.fk3001(pos));
% 
% zero = [0; 0; 0];
% disp(pp.fk3001(zero,1));
% disp(pp.fk3001(zero,2));
% disp(pp.fk3001(zero,3));
% pp.servo_jp(zero);
% 
% pause(1);
% var = pp.measured_cp();
% disp(var);
% 
% disp('pp goal cp');
% disp(pp.goal_cp());
% 
% disp('pp measured cp');
% disp(pp.measured_cp());

% zero = pp.fk3001([0; 0; 0;], 3)
% cal = pp.fk3001([90; -86; 33.1;], 3)
% p1 = pp.fk3001([0; 37.895; 32.864], 3)
% p2 = pp.fk3001([0; 0; -42.648], 3)
% 
% figure(1)
% pp.plot_arm([0; 0; 0;]);
% view(3)
% 
% figure(2)
% pp.plot_arm([-90; 86; 33.1;]);
% view(3)
% 
% figure(3)
% pp.plot_arm([0; 37.895; 32.864]);
% view(3)
% 
% figure(4)
% pp.plot_arm([0; 0; -42.648]);
% view(3)

pp.fk3001([-20; 12; -10;], 3)


pp.shutdown();  %Shutdown robot




