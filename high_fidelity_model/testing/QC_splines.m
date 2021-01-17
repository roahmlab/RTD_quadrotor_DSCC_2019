%% user parameters
% QC (quadcopter) initial conditions
p0 = [0;0;0] ; % position in x,y,z (leave this at the origin)
v0 = [1;0;0] ; % speed
a0 = [2;0;0] ; % acceleration

% desired future position
pf = [0;0;0] ;

% timing
time_horizon = 3 ; % s
sample_time = 0.05 ; % s

%% automated from here
% create a time vector
t = 0:sample_time:time_horizon ;

% desired future speed and acceleration (QC stops at end of desired traj)
vf = [0;0;0] ;
af = [0;0;0] ;

% for each axis, compute the change in position/velocity/accel
Dp = pf - p0 - v0.*time_horizon - 0.5.*a0.*time_horizon^2 ;
Dv = vf - v0 - a0.*time_horizon ;
Da = af - a0 ;

[ax,bx,cx] = single_axis_params(Dp(1),Dv(1),Da(1),time_horizon) ;
[ay,by,cy] = single_axis_params(Dp(2),Dv(2),Da(2),time_horizon) ;
[az,bz,cz] = single_axis_params(Dp(3),Dv(3),Da(3),time_horizon) ;

% compute the trajectory in each dimension
sx = (ax/120).*t.^5 + (bx/24).*t.^4 + (cx/6).*t.^3 + (a0(1)/2).*t.^2 + v0(1).*t + p0(1) ;
sy = (ay/120).*t.^5 + (by/24).*t.^4 + (cy/6).*t.^3 + (a0(2)/2).*t.^2 + v0(2).*t + p0(2) ;
sz = (az/120).*t.^5 + (bz/24).*t.^4 + (cz/6).*t.^3 + (a0(3)/2).*t.^2 + v0(3).*t + p0(3) ;

%% plotting
figure(1) ; cla ; hold on ; axis equal ;

plot3(p0(1),p0(2),p0(3),'bo') ;
plot3(sx,sy,sz,'b')
plot3(pf(1),pf(2),pf(3),'bx') ;
view(3)

xlabel('x') ; ylabel('y') ; zlabel('z') ;
set(gca,'FontSize',15)

%% helper functions
function [a,b,c] = single_axis_params(Dp,Dv,Da,T)
    M = [720, -360*T, 60*T^2 ;
         -360*T, 168*T^2, -24*T^3 ;
         60*T^2, -24*T^3, 3*T^4] ;
         
    out = (1/T^5)*M*[Dp;Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end