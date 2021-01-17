%% user parameters
% input angular velocities
T_in = [0 1] ;
O_in = 7.*rand(3,2) - 2 ;

% timing
dt = 0.05 ;

% display
disp_on = true ;
save_gif = false ;
gif_filename = 'test_rand_rotation_2s.gif' ;

%% automated from here
% create time vector
t_vec = 0:dt:T_in(end) ;

% create initial condition
R0 = eye(3) ;

% setup for plotting
figure(1) ; cla ; hold on ;
if save_gif
    start_gif = true ;
end

%% run fixed-step integration
% create initial conditions for Euler, Heun, and RK4
R1 = R0 ;
R2 = R0 ;
R4 = R0 ;

% set up plot
plot_SE3_body_frame(R0,zeros(3,1),'Colors',0.8.*ones(3),'LineWidth',1,'LineStyle','--') ;
plot_1 = plot_SE3_body_frame(R0,'Colors',[ones(3,1),zeros(3,2)],'LineWidth',2) ;
plot_2 = plot_SE3_body_frame(R0,'Colors',[zeros(3,2),ones(3,1)],'LineWidth',2) ;
plot_4 = plot_SE3_body_frame(R0,'Colors',[0.7.*ones(3,2),zeros(3,1)],'LineWidth',2) ;
axis([-1 1 -1 1 -1 1])
grid on

% run loop
for idx = 1:(length(t_vec)-1)
    % get time
    t_idx = t_vec(idx) ;
    
    % compute RK1 rotation matrix
    O1 = match_trajectories(t_idx,T_in,O_in) ;
    F1 = expm(dt.*skew(O1)) ;
    R1 = F1*R1 ;
    
    % compute RK2 rotation matrix
    % using section 2.1 of https://arxiv.org/pdf/1207.0069.pdf
    O2 = match_trajectories(t_idx + dt,T_in,O_in) ;
    F2 = expm(skew((dt/2)*(O1 + O2))) ;
    R2 = F2*R2 ;
    
    % compute RK4 rotation matrix
    % using pg 143. of http://hans.munthe-kaas.no/work/Blog/Entries/2000/5/4_Article__Lie_group_methods_files/iserles00lgm.pdf
    A1 = dt*skew(O1) ;
    B1 = A1 ;
    A2 = dt*skew(match_trajectories(t_idx + 0.5*dt,T_in,O_in)) ;
    B2 = A2 - A1 ;
    A3 = dt*skew(match_trajectories(t_idx + 0.5*dt,T_in,O_in)) ; % this is only because O is predefined
    B3 = A3 - A2 ;
    A4 = dt*skew(match_trajectories(t_idx + dt,T_in,O_in)) ;
    B4 = A4 - 2*A2 + A1 ;
    TH = B1 + B2 + (1/3)*B3 + (1/6)*B4 - (1/6)*bracket(B1,B2) - (1/12)*bracket(B1,B4) ;
    F4 = expm(TH) ;
    R4 = F4*R4 ;    
    
    % display info, check that rotation matrices are in SO(3)
    if disp_on
        disp('------------------------')
        disp(['ITERATION ',num2str(idx)])
        disp(['RK1 |R|:',num2str(det(R1))])
        disp('---')
        disp(['RK2 |R|:',num2str(det(R2))])
        disp('---')
        disp(['RK4 |R|:',num2str(det(R4))])
    end
    
    % plot
    fh = figure(1) ;
    plot_1 = plot_SE3_body_frame(R1,zeros(3,1),'Data',plot_1) ;
    plot_2 = plot_SE3_body_frame(R2,zeros(3,1),'Data',plot_2) ;
    plot_4 = plot_SE3_body_frame(R4,zeros(3,1),'Data',plot_4) ;
    axis equal
    axis([-1 1 -1 1 -1 1])
    view(3) ;
    pause(dt)
    
    % save gif
    if save_gif
        frame = getframe(fh) ;
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);

        if start_gif
            imwrite(imind,cm,gif_filename,'gif', 'Loopcount',inf,'DelayTime',dt) ; 
            start_gif = false ;
        else 
            imwrite(imind,cm,gif_filename,'gif','WriteMode','append') ; 
        end
    end
end

%% lie bracket on so(3)
function B = bracket(B1,B2)
    B = skew(B1*unskew(B2)) ;
end