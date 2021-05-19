time = csvread('time.csv');
lin = csvread('lin.csv');
ang = csvread('ang.csv');
mag = csvread('mag.csv');

figure(1)

subplot(3,1,1)
hold on;
plot(time, lin(:,1));
plot(time, lin(:,2));
plot(time, lin(:,3));
title('Linear Velocities vs. Time');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
legend('X direction', 'Y direction', 'Z direction');

hold off;

subplot(3,1,2)
hold on;
plot(time, ang(:,1));
plot(time, ang(:,2));
plot(time, ang(:,3));
title('Angular Velocities vs. Time');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('X direction', 'Y direction', 'Z direction');
hold off;

subplot(3,1,3)
hold on;
plot(time, mag);
title('Magnitude of Linear Velocity vs. Time');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');

hold off;


