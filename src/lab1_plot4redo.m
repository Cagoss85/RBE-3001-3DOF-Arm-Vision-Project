%Initializing arrays using the csv files
interpOneArray = csvread('interpOne.csv');
interpTwoArray = csvread('interpTwo.csv');
interpThreeArray = csvread('interpThree.csv');

noTerpOneArray = csvread('noTerpOne.csv');
noTerpTwoArray = csvread('noTerpTwo.csv');
noTerpThreeArray = csvread('noTerpThree.csv');

%Taking the data that we want from the arrays
timeInterpOne = interpOneArray(:,1);
interpTrialOne = interpOneArray(:,2);

timeInterpTwo = interpTwoArray(:,1);
interpTrialTwo = interpTwoArray(:,2);

timeInterpThree = interpThreeArray(:,1);
interpTrialThree = interpThreeArray(:,2);

timeNoTerpOne = noTerpOneArray(:,1);
noTerpTrialOne = noTerpOneArray(:,2);

timeNoTerpTwo = noTerpTwoArray(:,1);
noTerpTrialTwo = noTerpTwoArray(:,2);

timeNoTerpThree = noTerpThreeArray(:,1);
noTerpTrialThree = noTerpThreeArray(:,2);



%Subplot creation
figure(1)



%Subplot 1
subplot(3,1,1);
hold on
plot(timeInterpOne, interpTrialOne, 'Color', 'red');                                %plot joint pos vs time
plot(timeNoTerpOne, noTerpTrialOne, 'Color', 'blue');                             %plot joint pos vs time

title('Joint One Position vs Time, Trial One');    %add title to plot
xlabel('Time (seconds)');               %add x axis label
ylabel('Joint One Position');           %add y axis label
%axis([0 3 -5 45]);                      %specify window to compress noice / fix scale
legend('with interpolation', 'no interpolation')
hold off

%Subplot 2
subplot(3,1,2);
hold on
plot(timeInterpTwo, interpTrialTwo, 'Color', 'red');                                %plot joint pos vs time
plot(timeNoTerpTwo, noTerpTrialTwo, 'Color', 'blue');                             %plot joint pos vs time                        %add title to plot

title('Joint One Position vs Time, Trial Two');
xlabel('Time (seconds)');
ylabel('Joint One Position');
%axis([0 3 -5 45]);
legend('with interpolation', 'no interpolation')
hold off

%Subplot 3
subplot(3,1,3);
hold on
plot(timeInterpThree, interpTrialThree, 'Color', 'red');                              %plot joint pos vs time
plot(timeNoTerpThree, noTerpTrialThree, 'Color', 'blue');                           %plot joint pos vs time

title('Joint One Position vs Time, Trial Three');
xlabel('Time (seconds)');
ylabel('Joint One Position');
%axis([0 3 -5 45]);
legend('with interpolation', 'no interpolation')
hold off



