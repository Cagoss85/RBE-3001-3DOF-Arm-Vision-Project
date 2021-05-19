clc;
clear;
clear java;
format short;

DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;

myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

pp = Robot(myHIDSimplePacketComs);
% cam = Camera();
% 
% cam.DEBUG = DEBUG_CAM;

test = lab5imageTesting
