function predictor2( t, dt, la, lv, part )
theta = zeros(1, t/dt);
x = zeros(1, t/dt);
y = zeros(1, t/dt);
lv = zeros(1,t/dt);
la = zeros(1,t/dt);

for i=2:(t/dt)
    lv(i) = 0.25*sin(dt*i);
    %la(i)=sin(dt*i);
    theta(i) = theta(i-1) + la(i)*dt;
    x(i) = x(i-1) + lv(i)*dt*cos(theta(i));
    y(i) = y(i-1) + lv(i)*dt*sin(theta(i));
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
plot(la,'Color','g');
plot(lv,'Color','m');
legend('Angular Velocity','Linear Velocity');
xlabel('t');
ylabel('m/s, rad/s');
title(['Linear and Angular velocity for part ' part]);
end

