clear all

problem = prob.load('ship_viknes830.json', 'scenario.json');

trajectory = rrt_star.solve(problem);

plot(trajectory);