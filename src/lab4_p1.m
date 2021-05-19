syms L0 L1 L2 L3 q1 q2 q3 a1;
xPos = L3*cos(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2)) - L3*sin(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) + L2*cos(q1)*cos(q2 - pi/2) - L2*cos(a1)*sin(q1)*sin(q2 - pi/2);
yPos = L3*cos(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)) - L3*sin(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) + L2*cos(q2 - pi/2)*sin(q1) + L2*cos(a1)*cos(q1)*sin(q2 - pi/2);
zPos = L0 + L1 + L2*sin(a1)*sin(q2 - pi/2) + L3*sin(a1)*cos(q2 - pi/2)*sin(q3 + pi/2) + L3*sin(a1)*cos(q3 + pi/2)*sin(q2 - pi/2);

%Take derivative of position functions to get the derivatives of 
xVel = diff(xPos);
yVel = diff(yPos);
zVel = diff(zPos);

%"Velocity in (1) direction with respect to q(#)"
xq1 = diff(xPos, q1);
yq1 = diff(yPos, q1);
zq1 = diff(zPos, q1);

xq2 = diff(xPos, q2);
yq2 = diff(yPos, q2);
zq2 = diff(zPos, q2);

xq3 = diff(xPos, q3);
yq3 = diff(yPos, q3);
zq3 = diff(zPos, q3);


%Omega vaoes for each joint
w1 = [0;0;1];   %First one is always 0;0;1
% 3rd column of transformation from base to Frame 2
w2 = [sin(a1)*sin(q1); -sin(a1)*cos(q1); cos(a1)];
% 3d Column of transformation from base to tip. 
w3 = [  -cos(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) - sin(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2));
        -cos(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) - sin(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2));
        sin(a1)*cos(q2 - pi/2)*cos(q3 + pi/2) - sin(a1)*sin(q2 - pi/2)*sin(q3 + pi/2)];