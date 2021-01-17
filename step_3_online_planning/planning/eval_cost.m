function [cost, cost_grad] = eval_cost(v_peak, v_0, a_0, x_des, start_tic, timeout)

%     p_peak = [pos_quadrotor_peak(v_0(1), a_0(1), v_peak(1));...
%               pos_quadrotor_peak(v_0(2), a_0(2), v_peak(2));...
%               pos_quadrotor_peak(v_0(3), a_0(3), v_peak(3))];

    p_peak = (a_0./12.0) + (v_0./2.0) + (3.0.*v_peak./2.0) ;

    cost = sum((p_peak - x_des).^2);
    
    cost_grad = 3.0.*(p_peak - x_des)' ;
    
    if nargin > 4
        error_if_out_of_time(start_tic,timeout)
    end
end
