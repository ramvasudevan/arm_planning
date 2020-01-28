% sub = rossubscriber('/head_camera/rgb/image_raw') ;
% sub = rossubscriber('/head_camera/rbg/camera_info')
sub = rossubscriber('/head_camera/depth/image') ;
m = sub.receive()

%%
% start_tic = tic ;
% while toc(start_tic) < 10
    m = sub.receive() ;
    
    D = m.Data ;
    h = m.Height ;
    w = m.Width ;
    
    R = reshape(D(1:3:end),w,h)' ;
    G = reshape(D(2:3:end),w,h)' ;
    B = reshape(D(3:3:end),w,h)' ;
    I = cat(3,R,G,B) ;
    I_gray = rgb2gray(I) ;
    
    imshow(I) ;
%     toc(start_tic)
% end
%% NOTE TO SELF
% the camera parameters are:
% header: 
%   seq: 10177
%   stamp: 
%     secs: 1579641456
%     nsecs: 374652617
%   frame_id: "head_camera_rgb_optical_frame"
% height: 480
% width: 640
% distortion_model: "plumb_bob"
% D: [0.0, 0.0, 0.0, 0.0, 0.0]
% K: [567.9068849964351, 0.0, 326.4562886141607, 0.0, 551.8126031155958, 220.181341394646, 0.0, 0.0, 1.0]
% R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
% P: [567.9068849964351, 0.0, 326.4562886141607, 0.0, 0.0, 551.8126031155958, 220.181341394646, 0.0, 0.0, 0.0, 1.0, 0.0]
% binning_x: 0
% binning_y: 0
% roi: 
%   x_offset: 0
%   y_offset: 0
%   height: 0
%   width: 0
%   do_rectify: False
