%Assign Arrays to each members csv files
AmberArray = readmatrix('homeDataAmber.csv');
CaseyArray = readmatrix('homeDataCasey.csv');
AaronArray = readmatrix('homeDataAaron.csv');

%Assign figure properties
figure(1);
hold on;
view(3);
grid on;
title('Average Tip Position');
xlabel('x Position (mm)');
ylabel('y Position (mm)');
zlabel('z Position (mm)');

%Plot Ambers Data
scatter3(AmberArray(:,1),AmberArray(:,2),AmberArray(:,3), 'filled', 'm');
scatter3(mean(AmberArray(:,1)), mean(AmberArray(:,2)), mean(AmberArray(:,3)), '*m');

%Plot Caseys Data
scatter3(CaseyArray(:,1),CaseyArray(:,2),CaseyArray(:,3), 'filled', 'b');
scatter3(mean(CaseyArray(:,1)), mean(CaseyArray(:,2)), mean(CaseyArray(:,3)), '*b');

%Plot Aarons Data
scatter3(AaronArray(:,1),AaronArray(:,2),AaronArray(:,3), 'filled', 'g');
scatter3(mean(AaronArray(:,1)), mean(AaronArray(:,2)), mean(AaronArray(:,3)), '*g');

legend('Amber', 'Ambers Avg', 'Casey', 'Caseys Avg', 'Aaron', 'Aarons Avg');