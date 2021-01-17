function [] = generate_dynamics( t_plan, t_peak, t_total )
% computing and saving the quadcopter dynamics functions for FRS
% computation.
% v_peak is the only control parameter
% compute two dynamics functions: one for the time until v_peak, and one
% from v_peak to stopping behavior.

if nargin == 0
    t_plan = 0.5;
    t_peak = 1; % time to peak speed is one second.
    t_total = 3; % time of total trajectory.
end
t_to_stop = t_total - t_peak; % time from peak to stop
p0 = 0; % assume initial position is set as the origin.
a_peak = 0; % 0 acceleration at peak speed
v_f = 0; % 0 final speed
a_f = 0; % 0 final acceleration.

%% Compute "to peak" dynamics

syms p v_0 a_0 v_peak t; % variables for FRS: position, time. parameters for FRS: initial speed, initial acceleration, peak speed

% compute change in velocity/accel for each axis
Dv = v_peak - v_0 - a_0*t_peak ;
Da = a_peak - a_0 ;

% compute spline parameters
[ax, bx, cx] = single_axis_params(Dv,Da,t_peak);

% write velocity
dp = (ax/24).*t.^4 + (bx/6).*t.^3 + (cx/2).*t.^2 + (a_0).*t + v_0;

% write peak position
p_plan = (ax/120).*t_plan.^5 + (bx/24).*t_plan.^4 + (cx/6).*t_plan.^3 + (a_0(1)/2).*t_plan.^2 + v_0(1).*t_plan;
p_peak = (ax/120).*t_peak.^5 + (bx/24).*t_peak.^4 + (cx/6).*t_peak.^3 + (a_0(1)/2).*t_peak.^2 + v_0(1).*t_peak;

% write overall dynamics
x = [p; v_0; a_0; v_peak; t];
syms tdummy udummy

dx = [dp; 0; 0; 0; 1];
matlabFunction(dx, 'File', 'dyn_quadrotor_toPeak', 'vars', {tdummy x udummy});

%% Compute "to stop" dynamics

% for each axis, compute the change in velocity/accel
Dv = v_f - v_peak - a_peak*t_to_stop ;
Da = a_f - a_peak ;

% compute spline parameters
[ax, bx, cx] = single_axis_params(Dv,Da,t_to_stop);

% write velocity
dp = (ax/24).*t.^4 + (bx/6).*t.^3 + (cx/2).*t.^2 + (a_peak).*t + v_peak;

% write overall dynamics
x = [p; v_0; a_0; v_peak; t];
syms tdummy udummy

dx = [dp; 0; 0; 0; 1];
matlabFunction(dx, 'File', 'dyn_quadrotor_toStop', 'vars', {tdummy x udummy});

% compute the final position p_f as a function of v_peak
p_f = (ax/120).*t_to_stop.^5 + (bx/24).*t_to_stop.^4 + (cx/6).*t_to_stop.^3 + (a_peak/2).*t_to_stop.^2 + v_peak.*t_to_stop + p_peak;

% save position functions:
matlabFunction(p_plan, 'File', 'pos_quadrotor_plan', 'vars', {v_0 a_0 v_peak});
matlabFunction(p_peak, 'File', 'pos_quadrotor_peak', 'vars', {v_0 a_0 v_peak});
matlabFunction(p_f, 'File', 'pos_quadrotor_final', 'vars', {v_0 a_0 v_peak});

end

function [a,b,c] = single_axis_params(Dv,Da,T)
    M = [0 0 ;
         -12 6*T ;
         6*T -2*T^2] ;
         
    out = (1/T^3)*M*[Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end
   
