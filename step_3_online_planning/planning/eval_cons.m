function [c, ceq, gc, gceq] = eval_cons(x, v_0, v_max, a_max, t_peak, A_con, b_con, start_tic, timeout)
    epsilon = 1e-6 ;
    ceq = [];
    gceq = [] ;
    
    c1 = sum(x.^2) - v_max.^2 ;
    
    c2 = sum(((x - v_0)./t_peak).^2) - a_max.^2 ;

    c3 = A_con*x + b_con;
    c3 = reshape(c3, 6, []);
    [c3,i3] = min(c3, [], 1) ;
    c3 = c3 + epsilon ;
    
    c = [c1;c2;c3(:)] ;
    
    A = A_con(1:6,:) ;
    
    gc = [2.*x, (2.*(x-v_0)./t_peak^2), A(i3,:)'] ;

    if nargin > 7
        error_if_out_of_time(start_tic,timeout)
    end
end