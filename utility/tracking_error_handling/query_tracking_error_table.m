function T_log = query_tracking_error_table(T,v_cur,p_cur)
    T_log = (T.vx_lo <= v_cur(1)) & (T.vx_hi >= v_cur(1)) & ...
            (T.vy_lo <= v_cur(2)) & (T.vy_hi >= v_cur(2)) & ...
            (T.vz_lo <= v_cur(3)) & (T.vz_hi >= v_cur(3)) ;
        
    if nargin > 2
        T_log = T_log & (T.p_lo <= p_cur) & (T.p_hi >= p_cur) ;
    end
end