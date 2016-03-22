D = csvread('Beams_data_sim.txt');
R = csvread('Beams_data_lab.txt');
T = R(1:100,:);
G = D(1:100,:);
X = linspace(-0.513, 0.4999, 640);
Y = linspace(0, 10, 100);

X1 = linspace(-0.785398, 0.785398, 64);
Y1 = linspace(0, 10, 100);

figure,
mesh(X,Y,fliplr(T(:,(1:640))))
title('Beam data from physical robot')
xlabel('Beam angle (rad)')
ylabel('Time (s)')
zlabel('Distance (m)')

figure,
mesh(X1,Y1,fliplr(G(:,(1:64))))
title('Beam data from simulated robot')
xlabel('Beam angle (rad)')
ylabel('Time (s)')
zlabel('Distance (m)')