M_1= "giosim.csv";
X_1= csvread(M_1);

figure(1);
plot(X_1(:,1),X_1(:,2));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
title('Robot position - Simulation');


M_2= "amclData.csv";
X_2= csvread(M_2);

figure(2);
plot(X_2(:,1),X_2(:,2));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
title('Robot position - AMCL-Data during cartography');


M_3= "odometryData.csv";
X_3= csvread(M_3);

figure(3);
plot(X_2(:,1),X_2(:,2));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
title('Robot position - Odometry-Data during cartography');


M_4= "giocontroller.csv";
X_4= csvread(M_4);

figure(4);
plot(X_4(:,1),X_4(:,2));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
title('Robot position - Odometry-Data during path tracking');