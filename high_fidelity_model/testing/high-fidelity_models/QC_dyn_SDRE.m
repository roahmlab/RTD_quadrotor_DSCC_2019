%% notes
% Dynamic model from "Nonlinear State-Dependent Riccati Equation Control
% of a Quadrotor UAV" equation (9) with state-depenent Riccati equation for
% control.
%
% Paper available at: https://ieeexplore.ieee.org/document/4777039
%
% Parameters are in: https://ieeexplore.ieee.org/document/4957154
%
%% dynamics
function state_dot = QC_dyn_SDRE(time,state,T_input,U_input,params)
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
    
%% LOW LEVEL CONTROLLER START
    % get proportional controller gains for speed terms
    k1 = params.k1 ;
    k2 = params.k2 ;
    k3 = params.k3 ;  
    
    % create inertia coefficients
    I1 = (Iy-Iz)/Ix ;
    I2 = (Iz-Ix)/Iy ;
    I3 = (Ix-Iy)/Iz ;
    
    % get desired speed inputs (vx,vy,vz)_des
    U_t = interp1(T_input(:),U_input',time,'previous') ;
    x2d = U_t(1) ;
    x4d = U_t(2) ;
    x6d = U_t(3) ;
    
    % get artificial inputs
    u1tilde = k1*(x2d - x2) ;
    u2tilde = k2*(x4d - x4) ;
    u3tilde = k3*(x6d - x6) ;
    
    % generate u1 (which maintains a hover) and desired attitude (which
    % move the QC around)
    u1 = m*(u3tilde + g) ;
    x7d = (-u2tilde)/(u3tilde + g) ;
    x9d = u1tilde/(u3tilde + g) ;
    x11d = 0 ; % we don't need any desired yaw to move the QC around
    
    % use SDRE for attitude control about (x7d,x9d,x11d) to make
    % (u2,u3,u4), noting that wd (the measurable disturbance) is assumed to
    % be zero for our purposes, so we don't need to compute u_c
    AxI = [0 1 0 0 0 0 ; % from (16)
           0 0 0 x12*I1 0 0 ;
           0 0 0 1 0 0 ;
           0 x12*I2 0 0 0 0 ;
           0 0 0 0 0 1 ;
           0 0 0 x8*I3 0 0] ;
      
    BxI = [0 0 0 ; % from (16)
           l/Ix 0 0 ;
           0 0 0 ;
           0 l/Iy 0 ;
           0 0 0 ;
           0 0 l/Iz] ;
      
    % solve ricatti equation
    Q = diag([10 10 10 1 1 1]) ;
    [~,~,KxI] = care(AxI,BxI,Q) ;
    
    % compute pre-filter matrix (18)
    MxI = pinv(pinv(BxI*KxI - AxI)*BxI) ;
    
    % get attitude states and desired attitude states
    xI = state(7:end) ;
    xId = [x7d ; 0 ; x9d ; 0 ; x11d ; 0] ;
    
    % compute attitude control input (19)
    uI = -KxI*xI + MxI*xId ; % + u_c = 0
    u2 = uI(1) ;
    u3 = uI(2) ;
    u4 = uI(3) ;
    
%% LOW LEVEL CONTROLLER END
    
    % compute desired rotors speeds from (u1,...,u4) assuming that the
    % rotor speeds can be computed instantaneously
    
    % compute u1,...u4 actual based on saturated rotor speeds
    
    % compute dynamics (wd = 0 )
    state_dot = [x2 ;
                 (cos(x7)*sin(x9)*cos(x11) + sin(x7)*sin(x11))*u1/m ;
                 x4 ;
                 (cos(x7)*sin(x9)*sin(x11) - sin(x7)*cos(x11))*u1/m ;
                 x6 ;
                 ((cos(x7)*cos(x9))*u1/m) - g ;
                 x8 ;
                 x12*x10*I1 + (l/Ix)*u2 ;
                 x10 ;
                 x12*x8*I2 + (l/Iy)*u3 ;
                 x12 ;
                 x12*x8*I3 + (l/Iz)*u4 ] ;
             
    % % dynamics with disturbance:
    % state_dot = [x2 ;
    %              (cos(x7)*sin(x9)*cos(x11) + sin(x7)*sin(x11))*u1/m ;
    %              x4 ;
    %              (cos(x7)*sin(x9)*sin(x11) - sin(x7)*cos(x11))*u1/m ;
    %              x6 ;
    %              -g + ((cos(x7)*cos(x9))*u1/m) ;
    %              x8 ;
    %              x12*x10*I1 - (JR/Ix)*x10*wd + (l/Ix)*u2 ;
    %              x10 ;
    %              x12*x8*I2 + (JR/Iy)*x8*wd + (l/Iy)*u3 ;
    %              x12 ;
    %              x12*x8*I3 + (l/Iz)*u4 ] ;
end