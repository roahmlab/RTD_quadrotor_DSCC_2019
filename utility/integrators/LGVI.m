function [tout,Rout,Xout,Pout,Gout] = LGVI(input_fn,R0,X0,P0,G0,tspan)

% allocate outputs
tout = tspan ;
Nt = length(tout) - 1 ; % number of time steps to integrate
Rout = [R0, nan(3,Nt*3)] ;
Xout = [X0, nan(3,Nt)] ;
Pout = [P0, nan(3,Nt)] ;

%

end