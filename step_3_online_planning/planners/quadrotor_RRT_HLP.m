% classdef quadrotor_RRT_HLP < RRT_HLP
classdef quadrotor_RRT_HLP < RRT_star_HLP
    % Class: quadrotor_RRT_HLP < RRT_star_HLP
    %
    % This implements RRT* for the quadrotor (meaning that it
    % checks collision with 3D cuboid obstacles)
    %
    % Author: Shreyas Kousik
    % Created: Nov 2019
    % Updated: 23 Dec 2019
    
    properties
        buffer = 0 ;
        edge_feasibility_check_discretization = 0.01 ; 
        O_cell ; % cell array for obstacles
    end
    
    methods
        %% constructor
        function HLP = quadrotor_RRT_HLP(varargin)
%             HLP@RRT_HLP(varargin{:}) ;
            HLP@RRT_star_HLP(varargin{:}) ;
        end
        
        function setup(HLP,agent_info,world_info)
%             setup@RRT_HLP(HLP,agent_info,world_info) ;
            setup@RRT_star_HLP(HLP,agent_info,world_info) ;
            
            % set the bounds from the buffer
            b = HLP.buffer ;
            HLP.bounds = world_info.bounds - b.*[-1 1 -1 1 -1 1] ; % 3D
        end
        
        %% tree growth
        function exit_flag = grow_tree(HLP,agent_info,world_info)
            % update the HLP obstacles (this speeds up the edge feas check)
            O = world_info.obstacles ;
            b = HLP.buffer ;
            
            % get just the non-dynamic obstacles
            O_static = {} ;
            for idx = 1:length(O)
                o = O{idx} ;
                if isa(o,'zonotope_obstacle')
                    O_static = [O_static, {o}] ;
                end
            end
            
            % get the representation of the obstacles needed for the HLP
            if length(O_static) > length(HLP.O_cell)
                HLP.O_cell = cellfun(@(o) [o.center ; (2*b + o.body_dimensions(:))],...
                    O_static,'UniformOutput',false) ;
            end
            
            % call the superclass method
            exit_flag = grow_tree@RRT_star_HLP(HLP,agent_info,world_info) ;
        end
        
        %% node feasibility check
        function out = edge_feasibility_check(HLP,node_A,node_B,~)
            % this function should return TRUE if the node is feasible
            
            % get the line between the two nodes
            l = [node_A, node_B] ;
            d_min = HLP.edge_feasibility_check_discretization ; % 1 cm
            d = vecnorm(l(:,2) - l(:,1)) ;
            
            if d > 0
                N = ceil(d/d_min) ;
                d_des = linspace(0,d,N) ;
                L = match_trajectories(d_des,[0 d],l) ;

                % check the obstacles
                O_chk = cellfun(@(o) dist_point_to_box(L,o(4),o(5),o(6),o(1:3)) == 0,...
                    HLP.O_cell,'UniformOutput',false) ;

                % check all the obstacles
                out = ~any(cell2mat(O_chk)) ;
            else
                out = false ;
            end
        end
    end
end