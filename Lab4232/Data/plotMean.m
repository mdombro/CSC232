M = csvread('Mean_data.txt');
A = csvread('Pos_data.txt');
figure,
plot(M(:,1),M(:,2));
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
plot(linspace(0,s(1)/10,s(1)),M(:,1:2))
xlabel('Time (s)')
ylabel('Position (m)')
title('EKF predicted position vs time')
legend('X', 'Y')

figure,
s = size(A);
plot(linspace(0,s(1)/10,s(1)),A(:,1:2))
xlabel('Time (s)')
ylabel('Position (m)')
title('Actual position vs time')
legend('X','Y')


