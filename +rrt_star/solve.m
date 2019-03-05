function trajectory = solve(problem)
    p0 = problem.scenario.start_position;
    pf = problem.scenario.goal_position;
    xx0 = [p0; zeros(4,1)];
    xxf = [pf; zeros(4,1)];
    tree = rrt_star.Tree(xx0, xxf);
    
    
    % Test
    PLOT(problem, tree);
    SAVE(true)
            
    for i=2:1000
        xx_i = SAMPLE_RANDOM(problem);
        
        % Test
        PLOT(problem, tree);
        plot(xx_i(2), xx_i(1), 'b*', 'linewidth', 2.0);
        drawnow
        SAVE(false)
        
        [n_closest, traj, c_star] = CLOSEST_FEASIBLE_NODE_BACKWARD(problem, tree, xx_i);
        if (n_closest > 0)
            
            [tree, n_i] = tree.add(xx_i,n_closest,tree.nodes(n_closest).cost+c_star,traj);

            PLOT(problem, tree);
            SAVE(false)
            
            % Rewire
            tree = REWIRE(problem, tree, n_i);
            
            PLOT(problem, tree);
            SAVE(false)
        end
    end
    
    trajectory = [];
end

function PLOT(problem, tree)
    clf
    plot(problem.scenario);
    tree.plot();
    drawnow
end

function SAVE(first)
    filename = 'testAnimated.gif';
    % Capture the plot as an image 
    %h = figure;
  frame = getframe(); 
  im = frame2im(frame); 
  [imind,cm] = rgb2ind(im,256); 
  % Write to the GIF File 
  if first 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
  else 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
  end 
end

function xx = SAMPLE_RANDOM(problem)
    x_lim = problem.scenario.x_limit;
    y_lim = problem.scenario.y_limit;
    psi_lim = [0 2*pi*rand(1,1)];
    u_lim = [4 6];
    v_lim = [0 0];
    r_lim = [0 0];
    
    occupied = true;
    while occupied
       xx = [ x_lim(1)+(x_lim(2)-x_lim(1))*rand(1,1);
                y_lim(1)+(y_lim(2)-y_lim(1))*rand(1,1);
                psi_lim(1)+(psi_lim(2)-psi_lim(1))*rand(1,1);
                u_lim(1)+(u_lim(2)-u_lim(1))*rand(1,1);
                v_lim(1)+(v_lim(2)-v_lim(1))*rand(1,1);
                r_lim(1)+(r_lim(2)-r_lim(1))*rand(1,1) ];
       occupied = problem.scenario.is_collision(xx(1), xx(2)); 
    end
end

function [best_node_index, best_trajectory, c_star] = CLOSEST_FEASIBLE_NODE_BACKWARD(problem, tree, xx_i)
    R = eye(3)*0.00000000001;
    r = Inf;
    best_node_index = 0;
    best_trajectory = [];
    best_cost = Inf;
    for n=1:length(tree.nodes)
        xx = tree.nodes(n).xx;
        [trajectory, c_star] = rrt_star.optimal_trajectory(problem.ship, R, xx, xx_i);
        cost = tree.nodes(n).cost + c_star;
        if (~problem.scenario.is_trajectory_collision(trajectory))
            if (c_star < r && cost < best_cost)
                best_node_index = n;
                best_cost = cost;
                best_trajectory = trajectory;
            end
        end
    end
end

function tree = REWIRE(problem, tree, n_i)
    xx_i = tree.nodes(n_i).xx;
    R = eye(3)*0.00000000001;
    r = Inf;
    for n=1:length(tree.nodes)
        xx = tree.nodes(n).xx;
        [trajectory, c_star] = rrt_star.optimal_trajectory(problem.ship, R, xx_i, xx);
        cost = tree.nodes(n_i).cost + c_star;
        if (~problem.scenario.is_trajectory_collision(trajectory))
            if (c_star < r && cost < tree.nodes(n).cost)
                tree.nodes(n).cost = cost;
                tree.nodes(n).parent = n_i;
                tree.nodes(n).trajectory = trajectory;
            end
        end
    end
    % Endpoint
    xxf = tree.final_node.xx;
    [trajectory, c_star] = rrt_star.optimal_trajectory(problem.ship, R, xx_i, xxf);
    cost = tree.nodes(n_i).cost + c_star;
    if (~problem.scenario.is_trajectory_collision(trajectory))
        if (c_star < r && cost < tree.final_node.cost)
            tree.final_node.cost = cost;
            tree.final_node.parent = n_i;
            tree.final_node.trajectory = trajectory;
        end
    end
end


