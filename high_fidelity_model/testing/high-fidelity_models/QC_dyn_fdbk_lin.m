%% notes
% Dynamic model from "Nonlinear Control of a Quadrotor Micro-UAV using
% Feedback Linearization" equation (10).
%
% Paper available at: https://ieeexplore.ieee.org/document/4957154
%
%% dynamics function
function state_dot = QC_dyn_fdbk_lin(time,state,T_input,U_input,params)
%% setup
    % extract states (x,xd,y,yd,z,zd,phi,phid,theta,thetad,psi,psid); note
    % this is in a different order from the states in the paper, so the
    % numbers of the states (i.e. x#) are written to reflect the numbers in
    % the paper
    x1 = state(2) ; % dx/dt
    x2 = state(4) ; % dy/dt
    x3 = state(6) ; % dz/dt
    x4 = state(7) ; % phi
    x7 = state(8) ; % dphi/dt
    x5 = state(9) ; % theta
    x8 = state(10) ; % dtheta/dt
    x6 = state(11) ; % psi
    x9 = state(12) ; % dpsi/dt
    
    % extract params
    g = params.g ;
    m = params.m ;
    L = params.l ;
    Ix = params.Ix ;
    Iy = params.Iy ;
    Iz = params.Iz ;
%     b = params.b ; % propellor efficiency
%     d = params.d ; % drag factor
    
    % get proportional controller gains for speed terms
    k1 = params.k1 ;
    k2 = params.k2 ;
    k3 = params.k3 ;
    
    % get inner loop controller gains for attitude terms
    K2 = params.K2 ;
    K3 = params.K3 ;
    K4 = params.K4 ;
    w2 = params.w2 ; % can be chosen as w_i = (K_i/2)^2 * I_(x,y,z)/L
    w3 = params.w3 ;
    w4 = params.w4 ;
    
    % create inertia coefficients
    I1 = (Iy-Iz)/Ix ;
    I2 = (Iz-Ix)/Iy ;
    I3 = (Ix-Iy)/Iz ;
    
%% outer (velocity control) loop
    % get desired speed inputs (vx,vy,vz)_des
    U_t = interp1(T_input(:),U_input',time,'previous') ;
    x1d = U_t(1) ;
    x2d = U_t(2) ;
    x3d = U_t(3) ;
    
    % get artificial inputs
    u1tilde = k1*(x1d - x1) ;
    u2tilde = k2*(x2d - x2) ;
    u3tilde = k3*(x3d - x3) ;
    
    % generate u1 (which maintains a hover) and desired attitude (which
    % move the QC around)
    x6d = 0 ;
    if u1tilde ~=0
        bt = sqrt(((g-u3tilde)/u1tilde)^2 + 1) ; % (28)
        u1 = m*sqrt((u1tilde^2)/bt + u2tilde^2) ; % (29)
        al = u2tilde*m/u1 ; % (28)
        x4d = asin(al) ;
        x5d = -sign(u1tilde)*asin(bt) ;
    else
%% SHREY: LEFT OFF HERE 11 Feb 4:15 PM
        x4d = 0 ;
    end
    
    
%% inner (attitude control) loop
    % compute new inputs from (14)
    u2star = w2*(x4d - x4) ;
    u3star = w3*(x5d - x5) ;
    u4star = w4*(x6d - x6) ;
    
    % compute f2, f3, f4
    f2 = (Ix/L)*(K2*x7 - x8*x9*I1) ;
    f3 = (Iy/L)*(K3*x8 - x7*x9*I2) ;
    f4 = Iz*(K4*x9 - x7*x8*I3) ;
    
    % compute u2, u3, u4
    u2 = f2 + u2star ;
    u3 = f3 + u3star ;
    u4 = f4 + u4star ;
    
%% dynamics
    % compute dynamics (with wd = 0)
    state_dot = [x1 ;
                 (cos(x4)*sin(x5)*cos(x6) + sin(x4)*sin(x6))*u1/m ;
                 x2 ;
                 (cos(x4)*sin(x5)*sin(x6) - sin(x4)*cos(x6))*u1/m ;
                 x3 ;
                 -g + ((cos(x4)*cos(x5))*u1/m) ;
                 x7 ;
                 x9*x8*I1 + (L/Ix)*u2 ;
                 x8 ;
                 x9*x7*I2 + (L/Iy)*u3 ;
                 x9 ;
                 x9*x7*I3 + (L/Iz)*u4 ] ;
end