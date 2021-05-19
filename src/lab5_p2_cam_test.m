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
cam = Camera();

cam.DEBUG = DEBUG_CAM;
%% Main Code

basePnts = [];
cam.cam_pose = cam.getCameraPose();

disp('place Balls');
pause(5);

[img, newParams] = cam.undistortSnapshot();
img1 = cam.snapshot();

figure(1);
imshow(img1);
[x,y] = ginput(2);
close(figure(1));

posInterest = [x y];

for i=1:size(posInterest,1)
    pBase = cam.cam2Base(posInterest(i,:), true);
    basePnts = cat(1,basePnts, transpose(pBase(1:2,:)));
end
disp(basePnts);

%% %% Shutdown Procedure
pp.shutdown();
cam.shutdown();

