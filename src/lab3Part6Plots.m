
%establishing data for the graphs 
time = csvread('time.csv');
TSE = csvread('TSE.csv');
JAE = csvread('JAE.csv');

TSX = TSE(:,1);
TSY = TSE(:,2);
TSZ = TSE(:,3);

TSXV = gradient(TSX);
TSYV = gradient(TSY);
TSZV = gradient(TSZ);

TSXA = gradient(TSXV);
TSYA = gradient(TSYV);
TSZA = gradient(TSZV);

JA1 = JAE(:,1);
JA2 = JAE(:,2);
JA3 = JAE(:,3);

%plotting the graphs
figure(1)
hold on;

plot3(70, 70, 95, 'o');
plot3(100, 0, 150, 'o');
plot3(180, -50, 60, 'o');

plot3([70,100], [70,0], [95,150], 'r-', 'LineWidth', 2);  
plot3([100,180], [0,-50], [150,60], 'r-', 'LineWidth', 2);  
plot3([180,70], [-50,70], [60,95], 'r-', 'LineWidth', 2); 

title('Triangle Motion');
xlabel('X mm');
ylabel('Y mm');
zlabel('Z mm');
grid on;

hold off;
view(3);


figure(2)
plot3(JA1, JA2, JA3);
title('Robot Joint Angle Motion');
xlabel('Joint 1 Angle (degrees)');
ylabel('Joint 2 Angle (degrees)');
zlabel('Joint 3 Angle (degrees)');
grid on;

figure(3)
plot3(TSX, TSY, TSZ);
title('Robot Task Space Motion');
xlabel('X mm');
ylabel('Y mm');
zlabel('Z mm');
grid on;

figure(4) 

subplot(3,1,1)
hold on
plot(time, TSX) 
plot(time, TSY)
plot(time, TSZ)
title('Position over Time');
ylabel('mm');
xlabel('Time (s)');
legend('X','Y','Z');
hold off

subplot(3,1,2)
hold on
plot(time, TSXV) 
plot(time, TSYV)
plot(time, TSZV)
title('Velocity over Time');
ylabel('mm/s');
xlabel('Time (s)');
legend('X','Y','Z');
hold off

subplot(3,1,3)
hold on
plot(time, TSXA) 
plot(time, TSYA)
plot(time, TSZA)
title('Acceleration over Time');
ylabel('mm/s^2');
xlabel('Time (s)');
legend('X','Y','Z');
hold off









