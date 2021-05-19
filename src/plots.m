interpArray = csvread('interp.csv');
noInterpArray = csvread('noInterp.csv');


%Setting up different data columns
timeInterp = interpArray(:,1);              %sort out first column for time
interpJointOne = interpArray(:,2);          %Extract data for each joint
interpJointTwo = interpArray(:,3);
interpJointThree = interpArray(:,4);

timeNoInterp = noInterpArray(:,1);
noInterpJointOne = noInterpArray(:,2);
noInterpJointTwo = noInterpArray(:,3);
noInterpJointThree = noInterpArray(:,4);


%Subplot creation
figure(1)



%Subplot 1
subplot(3,1,1);
%hold on
plot(timeInterp, interpJointOne, 'Color', 'red');                                %plot joint pos vs time
%plot(timeNoInterp, noInterpJointOne, 'Color', 'blue');                             %plot joint pos vs time

title('Joint One Position vs Time');    %add title to plot
xlabel('Time (seconds)');               %add x axis label
ylabel('Joint One Position');           %add y axis label
%axis([0 3 -5 45]);                      %specify window to compress noice / fix scale
%legend('with interpolation', 'no interpolation')
%hold off

%Subplot 2
subplot(3,1,2);
%hold on
plot(timeInterp, interpJointTwo, 'Color', 'red');                                %plot joint pos vs time
%plot(timeNoInterp, noInterpJointTwo, 'Color', 'blue');                             %plot joint pos vs time                        %add title to plot

title('Joint Two Position vs Time');
xlabel('Time (seconds)');
ylabel('Joint Two Position');
%axis([0 3 -5 45]);
%legend('with interpolation', 'no interpolation')
%hold off

%Subplot 3
subplot(3,1,3);
%hold on
plot(timeInterp, interpJointThree, 'Color', 'red');                              %plot joint pos vs time
%plot(timeNoInterp, noInterpJointThree, 'Color', 'blue');                           %plot joint pos vs time

title('Joint Three Position vs Time');
xlabel('Time (seconds)');
ylabel('Joint Three Position');
%axis([0 3 -5 45]);
%legend('with interpolation', 'no interpolation')
%hold off