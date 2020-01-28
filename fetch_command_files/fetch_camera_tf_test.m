%% fetch wrrrrld stuff
A = robot_arm_3D_fetch() ;
W = fetch_base_world_static() ;
W.setup(A.get_agent_info)

%% ros stuff
% get ros transform tree
tf = rostf ;

% get transform from tag to camera
t_tag_to_cam = getTransform(tf,'tag_0','head_camera_rgb_optical_frame', 'Timeout',1) ;

% get transform from camera to base_link
t_cam_to_bln = getTransform(tf,'head_camera_rgb_optical_frame','base_link','Timeout',1) ;

%%
q_tag_to_cam = t_tag_to_cam.Transform.Rotation ;
R_tag_to_cam = quat2rotm([q_tag_to_cam.W q_tag_to_cam.X q_tag_to_cam.Y q_tag_to_cam.Z]) ;
T_tag_to_cam = t_tag_to_cam.Transform.Translation ;
T_tag_to_cam = [T_tag_to_cam.X ; T_tag_to_cam.Y ; T_tag_to_cam.Z] 

% get rotation from camera to baselink
q_cam_to_bln = t_cam_to_bln.Transform.Rotation ;
R_cam_to_bln = quat2rotm([q_cam_to_bln.W q_cam_to_bln.X q_cam_to_bln.Y q_cam_to_bln.Z]) ;
T_cam_to_bln = t_cam_to_bln.Transform.Translation ;
T_cam_to_bln = [T_cam_to_bln.X ; T_cam_to_bln.Y ; T_cam_to_bln.Z] ;

%%
dz = -R_cam_to_bln'*T_cam_to_bln ;

o = box_obstacle_zonotope('side_lengths',[0.3,0.3,0.3],'center',dz) ;

% plotting
figure(1) ; clf ; axis equal ; grid on ; view(3) ; hold on ;
plot(W) ;
plot(A)
plot(o)

%%
while true
%     try
        disp('trying to get transform')
        t_tag_to_cam = getTransform(tf,'tag_0','head_camera_rgb_optical_frame', 'Timeout',1) ;
        
        T_tag_to_cam = t_tag_to_cam.Transform.Translation ;
        T_tag_to_cam = [T_tag_to_cam.X ; T_tag_to_cam.Y ; T_tag_to_cam.Z]./20
        
        q_tag_to_cam = t_tag_to_cam.Transform.Rotation ;
        R_tag_to_cam = quat2rotm([q_tag_to_cam.W q_tag_to_cam.X q_tag_to_cam.Y q_tag_to_cam.Z]) ;
        
        disp('setting transform')
        R_test = eul2rotm([pi/2 0 0],'xyz') ;
        T_obs = R_tag_to_cam'*T_tag_to_cam
        dobs = R_test*T_obs ;
        
        disp('updating plot')
        V = o.plot_data.body.Vertices ;
        mV = mean(V,1) ;
        o.plot_data.body.Vertices = V - mV + repmat(dobs(:)',size(V,1),1) ;
%     catch
%     end
    pause(1)
end