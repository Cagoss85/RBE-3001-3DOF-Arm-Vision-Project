syms x(t) y(t) z(t) t;
R = 20;
h=2.5*pi;
t = 1:.5:8*pi;

x = 80 + R.*cos(t);
y = 80 + R.*sin(t);
z = 50 + h.*t;
figure(1);
plot3(x,y,z);
grid on;
title('Robot Spiral');
xlabel('x axis (mm)');
ylabel('y axis (mm)');
zlabel('z axis (mm)');
axis equal;
xlim([0 220]);
ylim([-140 180]);
zlim([0 195]);

x = transpose(x);
y = transpose(y);
z = transpose(z);
posData = [];
posData = cat(2,posData,x,y,z);
writematrix(posData,"spiral.csv");

