m = csvread('Mean_data.txt');

% yaw1 = atan2(2*(pos1(:,6).*pos1(:,5)+pos1(:,3).*pos1(:,4)), 1-2*(pos1(:,4).^2+pos1(:,5).^2));
% yaw2 = atan2(2*(pos2(:,6).*pos2(:,5)+pos2(:,3).*pos2(:,4)), 1-2*(pos2(:,4).^2+pos2(:,5).^2));
% yaw3 = atan2(2*(pos3(:,6).*pos3(:,5)+pos3(:,3).*pos3(:,4)), 1-2*(pos3(:,4).^2+pos3(:,5).^2));
% yaw4 = atan2(2*(pos4(:,6).*pos4(:,5)+pos4(:,3).*pos4(:,4)), 1-2*(pos4(:,4).^2+pos4(:,5).^2));
% yaw5 = atan2(2*(pos5(:,6).*pos5(:,5)+pos5(:,3).*pos5(:,4)), 1-2*(pos5(:,4).^2+pos5(:,5).^2));
yaw1 = atan2(2*(m(:,3).*m(:,6)+m(:,4).*m(:,5)), 1-2*(m(:,5).^2+m(:,6).^2));
yaw1 = yaw1(35:length(yaw1));

figure,
hold on;
plot(linspace(0,length(yaw1)/10,length(yaw1)), yaw1, 'Color','m');
legend('yaw');
title('Yaw of predicted robot');
ylabel('Radians');
xlabel('Time');

% figure,
% hold on;
% plot(twist1(:,1),'Color','r');
% plot(twist1(:,2),'Color','b');
% legend('Linear Velocity','Angular Velocity');
% title('Linear and angular velocity for Part 1');
% ylabel('m/s, rad/s');
% xlabel('Time');
% 
% 
% figure,
% hold on;
% plot(pos2(:,1),'Color','k');
% plot(pos2(:,2),'Color','g');
% plot(yaw2,'Color','m');
% legend('x','y', 'yaw');
% title('Position and yaw for Part 2');
% ylabel('meters, radians');
% xlabel('Time');
% figure,
% hold on;
% plot(twist2(:,1),'Color','r');
% plot(twist2(:,2),'Color','b');
% legend('Linear Velocity','Angular Velocity');
% title('Linear and angular velocity for Part 2');
% ylabel('m/s, rad/s');
% xlabel('Time');
% 
% 
% figure,
% hold on;
% plot(pos3(:,1),'Color','k');
% plot(pos3(:,2),'Color','g');
% plot(yaw3,'Color','m');
% legend('x','y', 'yaw');
% title('Position and yaw for Part 3');
% ylabel('meters, radians');
% xlabel('Time');
% figure,
% hold on;
% plot(twist3(:,1),'Color','r');
% plot(twist3(:,2),'Color','b');
% legend('Linear Velocity','Angular Velocity');
% title('Linear and angular velocity for Part 3');
% ylabel('m/s, rad/s');
% xlabel('Time');
% 
% figure,
% hold on;
% plot(pos4(:,1),'Color','k');
% plot(pos4(:,2),'Color','g');
% plot(yaw4,'Color','m');
% legend('x','y', 'yaw');
% title('Position and yaw for Part 4');
% ylabel('meters, radians');
% xlabel('Time');
% figure,
% hold on;
% plot(twist4(:,1),'Color','r');
% plot(twist4(:,2),'Color','b');
% legend('Linear Velocity','Angular Velocity');
% title('Linear and angular velocity for Part 4');
% ylabel('m/s, rad/s');
% xlabel('Time');
% 
% 
% figure,
% hold on;
% plot(pos5(:,1),'Color','k');
% plot(pos5(:,2),'Color','g');
% plot(yaw5,'Color','m');
% legend('x','y', 'yaw');
% title('Position and yaw for Part 5');
% ylabel('meters, radians');
% xlabel('Time');
% figure,
% hold on;
% plot(twist5(:,1),'Color','r');
% plot(twist5(:,2),'Color','b');
% legend('Linear Velocity','Angular Velocity');
% title('Linear and angular velocity for Part 5');
% ylabel('m/s, rad/s');
% xlabel('Time');
