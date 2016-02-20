pos1 = csvread('Pos_data 1.txt');
pos2 = csvread('Pos_data 2.txt');
pos3 = csvread('Pos_data 3.txt');
pos4 = csvread('Pos_data 4.txt');
pos5 = csvread('Pos_data 5.txt');
twist1 = csvread('Twist_data 1.txt');
twist2 = csvread('Twist_data 2.txt');
twist3 = csvread('Twist_data 3.txt');
twist4 = csvread('Twist_data 4.txt');
twist5 = csvread('Twist_data 5.txt');

yaw1 = atan2(2*(pos1(:,4).*pos1(:,3)+pos1(:,1).*pos1(:,2)), 1-2*(pos1(:,2).^2+pos1(:,3).^2));
yaw2 = atan2(2*(pos2(:,4).*pos2(:,3)+pos2(:,1).*pos2(:,2)), 1-2*(pos2(:,2).^2+pos2(:,3).^2));
yaw3 = atan2(2*(pos3(:,4).*pos3(:,3)+pos3(:,1).*pos3(:,2)), 1-2*(pos3(:,2).^2+pos3(:,3).^2));
yaw4 = atan2(2*(pos4(:,4).*pos4(:,3)+pos4(:,1).*pos4(:,2)), 1-2*(pos4(:,2).^2+pos4(:,3).^2));
yaw5 = atan2(2*(pos5(:,4).*pos5(:,3)+0        .*0        ), 1-2*(0        .^2+pos5(:,3).^2));

figure,
hold on;
plot(pos1(:,1),'Color','k');
plot(pos1(:,2),'Color','g');
plot(yaw1,'Color','m');
legend('x','y', 'yaw');
title('Position and yaw for Part 1');
ylabel('meters, radians');
xlabel('Time');
figure,
hold on;
plot(twist1(:,1),'Color','r');
plot(twist1(:,2),'Color','b');
legend('Linear Velocity','Angular Velocity');
title('Linear and angular velocity for Part 1');
ylabel('m/s, rad/s');
xlabel('Time');


figure,
hold on;
plot(pos2(:,1),'Color','k');
plot(pos2(:,2),'Color','g');
plot(yaw2,'Color','m');
legend('x','y', 'yaw');
title('Position and yaw for Part 2');
ylabel('meters, radians');
xlabel('Time');
figure,
hold on;
plot(twist2(:,1),'Color','r');
plot(twist2(:,2),'Color','b');
legend('Linear Velocity','Angular Velocity');
title('Linear and angular velocity for Part 2');
ylabel('m/s, rad/s');
xlabel('Time');


figure,
hold on;
plot(pos3(:,1),'Color','k');
plot(pos3(:,2),'Color','g');
plot(yaw3,'Color','m');
legend('x','y', 'yaw');
title('Position and yaw for Part 3');
ylabel('meters, radians');
xlabel('Time');
figure,
hold on;
plot(twist3(:,1),'Color','r');
plot(twist3(:,2),'Color','b');
legend('Linear Velocity','Angular Velocity');
title('Linear and angular velocity for Part 3');
ylabel('m/s, rad/s');
xlabel('Time');

figure,
hold on;
plot(pos4(:,1),'Color','k');
plot(pos4(:,2),'Color','g');
plot(yaw4,'Color','m');
legend('x','y', 'yaw');
title('Position and yaw for Part 4');
ylabel('meters, radians');
xlabel('Time');
figure,
hold on;
plot(twist4(:,1),'Color','r');
plot(twist4(:,2),'Color','b');
legend('Linear Velocity','Angular Velocity');
title('Linear and angular velocity for Part 4');
ylabel('m/s, rad/s');
xlabel('Time');


figure,
hold on;
plot(pos5(:,1),'Color','k');
plot(pos5(:,2),'Color','g');
plot(yaw5,'Color','m');
legend('x','y', 'yaw');
title('Position and yaw for Part 5');
ylabel('meters, radians');
xlabel('Time');
figure,
hold on;
plot(twist5(:,1),'Color','r');
plot(twist5(:,2),'Color','b');
legend('Linear Velocity','Angular Velocity');
title('Linear and angular velocity for Part 5');
ylabel('m/s, rad/s');
xlabel('Time');

