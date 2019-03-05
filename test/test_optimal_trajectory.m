clear all

ship = prob.ship.load('ship_viknes830.json');

%% Formulation
R = eye(3)*0.000000000001;
x0 = [0;0;0;0;0;0];
x1 = [1000;1000;pi;5;0;0];

[trajectory, c_star] = rrt_star.optimal_trajectory(ship, R, x0, x1);

plot(trajectory);