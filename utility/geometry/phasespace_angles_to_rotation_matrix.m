function R = phasespace_angles_to_rotation_matrix(yaw,pitch,roll)
% R = phasespace_angles_to_rotation_matrix(roll,pitch,yaw)
% R = phasespace_angles_to_rotation_matrix(yaw_pitch_roll_array)
%
% Return a 3-by-3 rotation matrix given roll, pitch, and yaw values from
% the phasespace motion capture coordinate frame. This rotation matrix is
% relative to a North-East-Up frame.
%
% NOTE! The roll, pitch, and yaw are expected to be given as Tait-Bryan
% angles, not Euler angles (they are extrinsic angles, not intrinsic angles
% in a rolling frame).
%
% The roll, pitch, and yaw inputs can be given as vectors, in which case
% the output is a 3-by-3-by-N array of rotation matrices.
%
% If only one input is given, it is assumed to be a 3-by-N array where each
% column is [yaw ; pitch ; roll] in the phasespace frame. The output is
% then a 3-by-3-by-N array of rotation matrices.

    if nargin == 1
        ypr = yaw ;
    else
        ypr = [yaw ; -pitch ; roll] ;
    end

    N = size(ypr,2) ;
    R = zeros(3,3,N) ;
    
    for idx = 1:N
        y = ypr(1,idx) ; p = ypr(2,idx) ; r = ypr(3,idx) ;
        
        Rx = eul2rotm([y 0 0],'xyz') ;
        Ry = eul2rotm([0 p 0],'xyz') ;
        Rz = eul2rotm([0 0 r],'xyz') ;
        R(:,:,idx) = Rx * Ry * Rz ;
    end
end