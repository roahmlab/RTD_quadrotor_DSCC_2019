%% notes
% Dynamic model from "Nonlinear State-Dependent Riccati Equation Control
% of a Quadrotor UAV" equation (9) with open loop inputs.
%
% Paper available at: https://ieeexplore.ieee.org/document/4777039
%
% Parameters are in: https://ieeexplore.ieee.org/document/4957154
%
%% dynamics
function state_dot = QC_dyn_open_loop(time,state,T_input,U_input,params)
    % extract states (x,xd,y,yd,z,zd,phi,phid,theta,thetad,psi,psid)
    x2 = state(2) ;
    x4 = state(4) ;
    x6 = state(6) ;
    x7 = state(7) ;
    x8 = state(8) ;
    x9 = state(9) ;
    x10 = state(10) ;
    x11 = state(11) ;
    x12 = state(12) ;
    
    
    % extract params
    g = params.g ;
    m = params.m ;
    l = params.l ;
    Ix = params.Ix ;
    Iy = params.Iy ;
    Iz = params.Iz ;
    JR = params.JR ;
    b = params.b ; % propellor efficiency
    d = params.d ; % drag factor
    
    % create inertia coefficients
    I1 = (Iy-Iz)/Ix ;
    I2 = (Iz-Ix)/Iy ;
    I3 = (Ix-Iy)/Iz ;
    
    % generate inputs (u1,u2,u3,u4,wd) - note that wd is a measurable
    % (gyroscopic) disturbance that we provide as an input for the open
    % loop model just for testing purposes
    U_t = interp1(T_input(:),U_input',time) ;
    u1 = U_t(1) ;
    u2 = U_t(2) ;
    u3 = U_t(3) ;
    u4 = U_t(4) ;
    wd = U_t(5) ;
    
    % compute dynamics
    state_dot = [x2 ;
                 (cos(x7)*sin(x9)*cos(x11) + sin(x7)*sin(x11))*u1/m ;
                 x4 ;
                 (cos(x7)*sin(x9)*sin(x11) - sin(x7)*cos(x11))*u1/m ;
                 x6 ;
                 -g + ((cos(x7)*cos(x9))*u1/m) ;
                 x8 ;
                 x12*x10*I1 - (JR/Ix)*x10*wd + (l/Ix)*u2 ;
                 x10 ;
                 x12*x8*I2 + (JR/Iy)*x8*wd + (l/Iy)*u3 ;
                 x12 ;
                 x12*x8*I3 + (l/Iz)*u4 ] ;
end