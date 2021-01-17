classdef QR_PID_LLC < low_level_controller
properties
    k_x = 1 ;
    k_v = 0 ;
    k_R = .1 ;
    k_O = 0 ;
end
methods
    %% constructor
    function LLC = QR_PID_LLC(varargin)
        LLC@low_level_controller
        LLC = parse_args(LLC,varargin) ;
    end

    %% get control inputs
    function U = get_control_inputs(LLC,A,t,z,varargin)
        % parse inputs
        T_ref = varargin{1} ;
        Z_ref = varargin{3} ;
        R = varargin{4} ;

        % get current desired trajectory
        z_d = match_trajectories(t,T_ref,Z_ref) ;
        
        % get agent properties
        e1 = A.e1 ;
        e3 = A.e3 ;
        m = A.m ;
        g = A.g*A.gravity_direction ;
        
        % get position error as in (17) [1]
        x = z(A.position_indices) ;
        x_d = z_d(1:3) ;
        e_x = x - x_d ;

        % get velocity error as in (18) [1]
        v = z(A.velocity_indices) ;
        v_d = z_d(4:6) ;
        e_v = v - v_d ;

        % get desired acceleration and thrust
        a_d = z_d(7:9) ;
        f_d = norm(a_d - g) ;
        
        % get b3d as in (23) [1]
        b3d_dir = -LLC.k_x*e_x - LLC.k_v*e_v - m*g + m*a_d ;
        b3d_norm = norm(b3d_dir) ;
        if b3d_norm > 1e-6
            b3d = b3d_dir./b3d_norm ;
        else
            b3d = e3 ;
        end

        % get b1d by projecting global first axis onto the plane normal to b3d
        b1d_default = e1 ;
        b1d_dir = b1d_default - (b1d_default'*b3d).*b3d ; % on plane normal to b3d
        b1d_norm = norm(b1d_dir) ;
        if b1d_norm > 1e-6
            b1d = b1d_dir./b1d_norm ;
        else
            b1d = [1;0;0] ;
        end

        % bet b2d from b1d and b3d
        b2d = cross(b3d,b1d) ;

        % get desired attitude
        R_d = [b1d b2d b3d] ;
        
        % get attitude error
        e_R = 0.5*unskew(R_d'*R - R'*R_d) ;
        
        % get desired angular velocity
        b3 = R(:,3) ;
        O_d = cross(b3,b3d) ;
        
        % compute body rate error
        O = z(A.angular_velocity_indices) ;
        e_O = O - O_d ;

        % compute desired moment as in (8) of [2]
        M_d = -LLC.k_R*e_R - LLC.k_O*e_O ;

        % output desired force and moment
        U = [A.m*f_d ; M_d] ;
    end
end
end