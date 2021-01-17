%% user parameters
N_points = 200 ;


%% automated from here
P = rand(N_points,3) ;

K = convhull(P) ;
I = unique(K) ;
length(I)
PC = P(I,:) ;


K2 = boundary(P) ;
I2 = unique(K2) ;
length(I2)
Q = P(I2,:) ;


figure(1) ; clf ; hold on ; axis equal ;
while length(I2) > length(I)
    
    K2 = boundary(Q) ;
    I2 = unique(K2) ;
    length(I2)
    Q = Q(I2,:) ;
    
    trisurf(K,P(:,1),P(:,2),P(:,3),'FaceColor','r','FaceAlpha',0.5)
    plot3(Q(:,1),Q(:,2),Q(:,3),'bx')
    pause(0.1)
end

%% plotting
figure(1) ; clf ; hold on ; axis equal ;
trisurf(K,P(:,1),P(:,2),P(:,3),'FaceColor','r','FaceAlpha',0.5)
plot3(Q(:,1),Q(:,2),Q(:,3),'bx')