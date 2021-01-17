function t = compute_timing_from_position_and_speed(position,speed)
    t = zeros(1,size(position,2)) ;
    dP = vecnorm(position(:,2:end) - position(:,1:end-1)) ;
    t(2:end) = cumsum(dP./speed) ;
end