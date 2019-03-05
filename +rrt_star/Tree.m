classdef Tree
    
    properties
        nodes
        final_node
    end
    
    methods
        function this = Tree(xx_start, xx_final)
            this.nodes = [];
            this.final_node.xx = xx_final;
            this.final_node.parent = 0;
            this.final_node.cost = Inf;
            this.final_node.trajectory = [];
            this = this.add(xx_start, 0, 0, []);
        end
        
        function [this, n] = add(this, xx, parent, cost, trajectory)
            node.xx = xx;
            node.parent = parent;
            node.cost = cost;
            node.trajectory = trajectory;
            this.nodes = [this.nodes node];
            n = length(this.nodes);
        end
        
        function plot(this)
            for n=1:length(this.nodes)
                this.plot_node(this.nodes(n), 'r', 'linewidth', 2.0);
            end
            % Final
            if (this.final_node.parent > 0)
                node = this.final_node;
                while node.parent > 0
                    this.plot_node(node, 'y', 'linewidth', 2.0);
                    node = this.nodes(node.parent);
                end
            end
        end
        
        function plot_node(this,node, varargin)
            plot(node.xx(2), node.xx(1), 'b*');
            if (~isempty(node.trajectory))
                plot(node.trajectory.y(:,2,1)', node.trajectory.y(:,1,1)', varargin{:});
            end
        end
    end
end

