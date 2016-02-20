function predictor( t, dt, la, lv, part )
theta = zeros(1, t/dt);
x = zeros(1, t/dt);
y = zeros(1, t/dt);

for i=2:41
    theta(i) = theta(i-1) + la*dt;
    x(i) = x(i-1) + lv*dt*cos(theta(i));
    y(i) = y(i-1) + lv*dt*sin(theta(i));
    %i = i + 1;
end

figure,
hold on
plot(theta, 'Color', 'k');
plot(x, 'Color', 'r')
plot(y, 'Color', 'b')
legend('theta','x','y');
xlabel('t')
ylabel('meters, rad')
title(['Position and theta vs t for part ' part]);

figure,
hold on
plot(linspace(la,la,40),'Color','g');
plot(linspace(lv,lv,40),'Color','m');
legend('Angular Velocity','Linear Velocity');
xlabel('t');
ylabel('m/s, rad/s');
title(['Linear and Angular velocity for part ' part]);
end

