function T_out = get_tracking_error_for_current_velocity(T,v_cur,get_all_rows)
    if nargin < 3
        get_all_rows = false ;
    end

    % figure out if x and y need to be reflected
    reflect_x = v_cur(1) < 0 ;
    reflect_y = v_cur(2) < 0 ;
    
    % when the vertical speed is exactly 0, there are two valid bins in the
    % tracking error table, causing an error
    if v_cur(3) < 0.01
        v_cur(3) = 0.01 ;
    end
    
    % get current tracking error
    v_query = [abs(v_cur(1:2)); v_cur(3)] ;
    T_log = query_tracking_error_table(T,v_query) ;
    
    if get_all_rows
        T_out = T(T_log,:) ;
    else
        T_out = T(T_log,9:end) ;
    end
    
    % reflect x and y if necessary
    if reflect_x
        ex_lo = -T_out.ex_hi ;
        ex_hi = -T_out.ex_lo ;
        T_out.ex_lo = ex_lo ;
        T_out.ex_hi = ex_hi ;
    end
    
    if reflect_y
        ey_lo = -T_out.ey_hi ;
        ey_hi = -T_out.ey_lo ;
        T_out.ey_lo = ey_lo ;
        T_out.ey_hi = ey_hi ;
    end
end