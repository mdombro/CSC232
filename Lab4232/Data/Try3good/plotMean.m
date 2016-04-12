M = csvread('Mean_data.txt');
M = M(35:length(M(:,3)),:);
A = csvread('Pos_data.txt');
m = csvread('Mean_data.txt');

yaw1 = atan2(2*(m(:,3).*m(:,6)+m(:,4).*m(:,5)), 1-2*(m(:,5).^2+m(:,6).^2));
yaw1 = yaw1(35:length(yaw1));
yaw2 = atan2(2*(A(:,6).*A(:,5)+A(:,3).*A(:,4)), 1-2*(A(:,4).^2+A(:,5).^2));

figure,
plot(M(:,1),M(:,2));
hold on
xlabel('X axis position (m)')
ylabel('Y axis position (m)')
title('EKF predicted motion')

figure,
plot(A(:,1),A(:,2));
title('Actual robot position')
xlabel('X axis position (m)')
ylabel('Y axis position (m)')

figure,
s = size(M);
subplot(2,1,1);
plot(linspace(0,s(1)/10,s(1)),M(:,1:2))
title('EKF predicted position vs time')
legend('X', 'Y')
xlabel('Time (s)')
ylabel('Position (m)')
subplot(2,1,2);
plot(linspace(0,length(yaw1)/10,length(yaw1)), yaw1, 'Color','m');
xlabel('Time (s)')
ylabel('Yaw (radians)')

figure,
s = size(A);
subplot(2,1,1);
plot(linspace(0,s(1)/10,s(1)),A(:,1:2))
xlabel('Time (s)')
ylabel('Position (m)')
title('Actual position vs time')
legend('X','Y')
subplot(2,1,2);
plot(linspace(0,length(yaw2)/10,length(yaw2)), yaw2, 'Color','m');
xlabel('Time (s)')
ylabel('Yaw (radians)')

