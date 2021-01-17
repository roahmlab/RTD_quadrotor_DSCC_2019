classdef Mellinger_LLC < low_level_controller
% Adapted from "Minimum Snap Trajectory Generation and Control for
% Quadrotors" by Daniel Mellinger and Vijay Kumar
%
% Author: Shreyas Kousik
% Created: some time in April 2019?
% Updated: 18 Sept 2020
    properties
        % these gain matrices were tuned by hand for the quadrotor_agent
        
        % for desired position trajectories
        %     K_p = 4.5*eye(3) ;
        %     K_v = 1.1*eye(3) ;
        %     K_R = 0.15*eye(3) ;
        %     K_w = 0.01*eye(3) ;
        
        % for desired velocity (peak speed) trajectories
        K_p = 2*eye(3) ;
        K_v = 0.5*eye(3) ;
        K_R = 1*eye(3) ;
        K_w = 0.03*eye(3) ;
    end
    
    methods
        %% constructor
        function LLC = Mellinger_LLC(varargin)
            LLC@low_level_controller(varargin{:}) ;
        end
        
        %% get control inputs
        function u = get_control_inputs(LLC,A,t,z,varargin)
            % method: U = get_control_inputs(LLC,A,t,z,T_ref,U_ref,Z_ref,R)
            
            % parse inputs
            T_ref = varargin{1} ;
            Z_ref = varargin{3} ;
            R = varargin{4} ;
            
            % get agent properties
            m = A.body_mass ;
            g = A.gravity_acceleration*A.gravity_direction ;
            
            % get current states
            p = z(A.position_indices) ;
            v = z(A.velocity_indices) ;
            w = z(A.angular_velocity_indices) ;
            
            % get current desired trajectory, which should have its states as
            % position, velocity, acceleration, jerk, and snap, written as
            % (x,v,a,j,s) \in R^15; note that the reference yaw is 0
            z_d = match_trajectories(t,T_ref,Z_ref) ;
            p_d = z_d(1:3) ;
            v_d = z_d(4:6) ;
            a_d = z_d(7:9) ;
            j_d = z_d(10:12) ;
            
            % desired body frame z axis
            e_p = p - p_d ;
            e_v = v - v_d ;
            F_d = -LLC.K_p*e_p - LLC.K_v*e_v - m*g + m*a_d ; % desired body force
            z_B_d = F_d/norm(F_d) ;
            
            % desired thrust force
            z_B = R(:,3) ;
            u_thrust = F_d'*z_B ; % desired net body force
            
            % desired world x axis assuming yaw ref is 0
            x_C_d = [1;0;0] ;
            
            % desired body y axis
            y_B_d_dir = cross(z_B_d,x_C_d) ;
            y_B_d = y_B_d_dir/norm(y_B_d_dir) ;
            
            % desired body x axis
            x_B_d = cross(y_B_d,z_B_d) ;
            
            % desired attitude
            R_d = [x_B_d,y_B_d,z_B_d] ;
            
            % orientation error
            e_R = (0.5).*unskew(R_d'*R - R'*R_d) ;
            
            % desired angular velocity
            h_w = (m/u_thrust)*(j_d - (z_B_d'*j_d)*z_B_d) ;
            p_d = -h_w'*y_B_d ;
            q_d = h_w'*x_B_d ;
            r_d = 0 ; % since desired yaw rate is 0
            w_d = [p_d;q_d;r_d] ;
            
            % angular velocity error
            e_w = w - w_d ;
            
            % compute desired moments
            u_moment = -LLC.K_R*e_R - LLC.K_w*e_w ;
            
            % output!
            u = [u_thrust ; u_moment] ;
        end
    end
end