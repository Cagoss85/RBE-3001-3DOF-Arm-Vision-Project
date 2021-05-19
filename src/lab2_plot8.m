%extracting data from the csv files and 
jointArray = csvread('angleData.csv');
timeAngle = jointArray(:,1);              %extract data for time 
jointOne = jointArray(:,2);          %Extract data for each joint
jointTwo = jointArray(:,3);
jointThree = jointArray(:,4);

tipPositionArray = csvread('XYZData.csv');
timePosition = tipPositionArray(:,1);
xArray = tipPositionArray(:,2);
yArray = tipPositionArray(:,3);
zArray = tipPositionArray(:,4);


%plotting data

%joint angle plot 
figure(1)

hold on;

plot(timeAngle, jointOne, 'Color', 'red');
plot(timeAngle, jointTwo, 'Color', 'blue');
plot(timeAngle, jointThree, 'Color', 'green');

title('Joint Angle vs. Time');
xlabel('Time (s)');
ylabel('Joint Angle (degrees)')
legend('Joint One', 'Joint Two', 'Joint Three');

hold off;

%X and Z over time position plot
figure(2)

hold on;

plot(timePosition, xArray, 'Color', 'red');
plot(timePosition, zArray, 'Color', 'blue');

title('Tip Position vs. Time');
xlabel('Time (s)');
ylabel('Position (mm)');
legend('X Position', 'Z Position');

hold off;

%X vs Z plot
figure(3)

hold on;

plot(xArray, zArray, 'Color', 'green');
plot(94.3759, 79.5, '.', 'Color', 'red');
plot(71, 264, '.', 'Color', 'red');
plot(150.5, 199, '.', 'Color', 'red');

title('Z Position vs. X Position');
xlabel('X Position (mm)');
ylabel('Z Position (mm)');

hold off;

%X vs Y plot
figure(4)

plot(xArray, yArray);

title('Y Position vs. X Position');
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
axis([70 160 -20 20]);                      %specify window to compress noice / fix scale














