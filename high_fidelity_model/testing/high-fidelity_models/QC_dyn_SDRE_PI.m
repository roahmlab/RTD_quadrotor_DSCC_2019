%% notes
% Dynamic model from "Nonlinear State-Dependent Riccati Equation Control
% of a Quadrotor UAV" equation (9) with state-depenent Riccati equation for
% control.
% 
% In addition to the implementation from the paper, we add (1) an integral
% controller to compensate for error and (2) input constraints, since LQR
% does not obey them.
%
% Paper available at: https://ieeexplore.ieee.org/document/4777039
%
% Parameters are in: https://ieeexplore.ieee.org/document/4957154
%
%% dynamics
function state_dot = QC_dyn_SDRE_PI(time,state,T_input,U_input,Z_input,params)
    % extract states (x,xd,y,yd,z,zd,phi,phid,theta,thetad,psi,psid)
    x1 = state(1) ;
    x2 = state(2) ;
    x3 = state(3) ;
    x4 = state(4) ;
    x5 = state(5) ;
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
%     JR = params.JR ;
%     b = params.b ; % propellor efficiency
%     d = params.d ; % drag factor
%     max_rotor_speed = params.max_rotor_speed ;
    
    % get proportional and integral controller gains for speed terms
    k1 = params.k1 ;
    k2 = params.k2 ;
    k3 = params.k3 ;
    ki = params.ki ;
    
    % create inertia coefficients
    I1 = (Iy-Iz)/Ix ;
    I2 = (Iz-Ix)/Iy ;
    I3 = (Ix-Iy)/Iz ;
    
    % get desired position speed, and accel inputs
    Z_t = interp1(T_input(:),Z_input',time,'previous') ;
    x1d = Z_t(1) ;
    x3d = Z_t(2) ;
    x5d = Z_t(3) ;
    
    U_t = interp1(T_input(:),U_input',time,'previous') ;
    x2d = U_t(1) ;
    x4d = U_t(2) ;
    x6d = U_t(3) ;
    
    % get artificial inputs   
    ex1 = x1d - x1 ;
    ex3 = x3d - x3 ;
    ex5 = x5d - x5 ;
    
    u1tilde = k1*(x2d - x2) + ki*ex1 ;
    u2tilde = k2*(x4d - x4) + ki*ex3 ;
    u3tilde = k3*(x6d - x6) + ki*ex5 ;
    
    % generate u1 (which maintains a hover) and desired attitude (which
    % move the QC around)
    u1 = m*(u3tilde + g) ;
    x7d = (-u2tilde)/(u3tilde + g) ;
    x9d = u1tilde/(u3tilde + g) ;
    x11d = 0 ; % we don't need any desired yaw to move the QC around
    
    % use SDRE for attitude control about (x7d,x9d,x11d) to make the inputs
    % (u2,u3,u4); note that wd (the measurable disturbance) is assumed to
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
      
    % solve riccati equation
    Q = diag([10 10 10 1 1 1]) ;
    [~,~,KxI] = care(AxI,BxI,Q) ;
    
    % compute pre-filter matrix (18)
    MxI = pinv(pinv(BxI*KxI - AxI)*BxI) ;
    
    % get attitude states and desired attitude states
    xI = state(7:12) ;
    xId = [x7d ; 0 ; x9d ; 0 ; x11d ; 0] ;
    
    % compute attitude control input (19)
    uI = -KxI*xI + MxI*xId ; % + u_c = 0
    u2 = uI(1) ;
    u3 = uI(2) ;
    u4 = uI(3) ;
    
%     % bound inputs approximately (this doesn't bound the individual rotor
%     % speeds, but it's ok for now)
%     u1max = b*4*max_rotor_speed^2 ;
%     uImax = b*max_rotor_speed^2 ;
%     
%     u1 = boundValues(u1,[0,u1max]) ;
%     uI = boundValues(uI,uImax) ;
%     u2 = uI(1) ;
%     u3 = uI(2) ;
%     u4 = uI(3) ;
    
    % compute dynamics (wd = 0 ) with error
    state_dot = [x2 ;
                 (cos(x7)*sin(x9)*cos(x11) + sin(x7)*sin(x11))*u1/m ;
                 x4 ;
                 (cos(x7)*sin(x9)*sin(x11) - sin(x7)*cos(x11))*u1/m ;
                 x6 ;
                 -g + ((cos(x7)*cos(x9))*u1/m) ;
                 x8 ;
                 x12*x10*I1 + (l/Ix)*u2 ;
                 x10 ;
                 x12*x8*I2 + (l/Iy)*u3 ;
                 x12 ;
                 x12*x8*I3 + (l/Iz)*u4] ;
             
%     % dynamics with disturbance:
%     state_dot = [x2 ;
%                  (cos(x7)*sin(x9)*cos(x11) + sin(x7)*sin(x11))*u1/m ;
%                  x4 ;
%                  (cos(x7)*sin(x9)*sin(x11) - sin(x7)*cos(x11))*u1/m ;
%                  x6 ;
%                  -g + ((cos(x7)*cos(x9))*u1/m) ;
%                  x8 ;
%                  x12*x10*I1 - (JR/Ix)*x10*wd + (l/Ix)*u2 ;
%                  x10 ;
%                  x12*x8*I2 + (JR/Iy)*x8*wd + (l/Iy)*u3 ;
%                  x12 ;
%                  x12*x8*I3 + (l/Iz)*u4 ] ;
end