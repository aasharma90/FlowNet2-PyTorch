% InterPolatePoses script generates relative poses of pose_timestamps w.r.t
% the origin timestamp using absoulte poses of INS sensor which are at INS
% timestamps w.r.t earth reference frame

% For every pose_timestamp entry (whether from LIDAR or camera), it
% computes an INS timestamp window such that
% ins_timestamp_n <= pose_timestamp entry < ins_timestamp_n+1
% n is diff. for every timestamp entry 

% It then obtains absolute rotational poses in terms of quaternions for 
% this upper and lower bound of INS timestamps, and then performs
% quaternion interpolation to get the quaternion at the pose_timestamp entry

% After it has absolute positions and quaternions at pose_timestamp
% entries from interpolation, it then computes the relative poses w.r.t
% origin timestamp in INS frame of reference

% Then doing the INS->Camera, Camera->Image transformations, relative pose
% solution b/w two frames in Image frame were computed. 


close all;
clear all;
addpath(genpath('flow_code'));


% Set Image dir
image_dir = '../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo/left';
% Set 2D LASER dir
laser_dir = '../../2014-12-10-18-10-50_lms_front_07/2014-12-10-18-10-50/lms_front';
% Set 3-D LASER dir
laser3d_dir = '../../2014-12-10-18-10-50_ldmrs_07/2014-12-10-18-10-50/ldmrs';
% Set INS file
ins_file = '../../2014-12-10-18-10-50_gps/2014-12-10-18-10-50/gps/ins.csv';
% Set models dir
models_dir = '../../camera-models';
% Set extrinsics dir
extrinsics_dir = '../extrinsics';
% Read timestamps
timestamps = dlmread('../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo.timestamps');


% Get intrinsics from ReadCameraModel 
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(image_dir, models_dir);

% Compute relative poses in INS frame of reference
check_frames = 2;
store_results = 0;
if(store_results == 1)
    vidObj1 = VideoWriter('OpticalFlow.avi');
    vidObj1.FrameRate = 4;
    open(vidObj1);
end
pose_timestamps = [];

for i=1:check_frames-1
    curr_frame = i;
    next_frame = i+1;
    pose_timestamp_curr = timestamps(end-2313+(curr_frame+100-1),1);
    pose_timestamp_next = timestamps(end-2313+(next_frame+100-1),1);
    pose_timestamps     = [pose_timestamp_curr pose_timestamp_next];
    
    % Investigate what InterpolatePoses function does
    [poses] = InterpolatePoses(ins_file, pose_timestamps, pose_timestamp_curr);
    
    % Transofrm the relative poses obtained w.r.t INS frame to relative poses
    % w.r.t camera frame and then to image frame
    camera_extrinsics = [0     0     0     0     0     0];
    ins_extrinsics    = [-1.7132    0.1181    1.1948   -0.0125    0.0400    0.0050];
    G_camera_ins = SE3MatrixFromComponents(camera_extrinsics) * SE3MatrixFromComponents(ins_extrinsics);
    for j=1:size(poses,1)
        % Move relative pose solution from INS frame of reference into camera frame of reference
        % To obtain :- P_cam2 to P_cam1
        poses{j} = (G_camera_ins)      * (poses{j}) *   inv(G_camera_ins);
        % Move relative pose solution from camera frame of reference into image frame of reference
        % To obtain :- P_image2 to P_image1
        poses{j} = inv(G_camera_image) * (poses{j}) *    (G_camera_image);
        % Inverse the transformation matrix obtained.  
        % To obtain :- P_image1 to P_image2.
        poses{j} = inv(poses{j});
    end
    
    % Obtain R and t from relative pose soln
    R1 = eye(3);
    T1 = [0; 0; 0];
    R2 = poses{2}(1:3,1:3);
    T2 = poses{2}(1:3, end);
    K1 = [fx    0    cx
          0     fy   cy
          0     0     1];
    K2 = [fx    0    cx
          0     fy   cy
          0     0     1];
    
    % Get [u,v,depth] data from PointCloud Projection
    [uv, colours, image] = ProjectLaserIntoCamera(image_dir, laser_dir, ins_file, models_dir, extrinsics_dir, pose_timestamp_curr);
    
    % Display result
    if(store_results == 0)
        figure(1), imshow(image);
        colormap jet;
        hold on;
        scatter(uv(:,1),uv(:,2), 90, 255.*(colours - min(colours(:)))./(max(colours(:)) - min(colours(:))), '.');
    end
    
    % Load Images
    image1 = LoadImage(image_dir, pose_timestamps(1), LUT);
    image2 = LoadImage(image_dir, pose_timestamps(2), LUT);
   
    % Check epipolar geometry 
    N = size(uv,1);
    X1 = zeros(3, N);
    X2 = zeros(3, N);
    z  = zeros(1, N);
    for n=1:N
        X1(:,n) = [uv(n,1); uv(n,2); 1];
        z(:,n)  = colours(n,1);
    end
    
    % Compute the corresponding points in the second image
    % Eqn - (K2 * R2 * R1 * inv(K1))*X1   +   (K2 * (T2 - T1)/Z)
    % Find corresponding points X2 in image2 
    X2(:,:) = ((K2 *  R2 * R1* inv(K1)) * X1) + repmat((K2 * (T2 - T1)), 1, N)./repmat(z, [3 1]);
    X2(:,:) = round(X2(:,:)./repmat(X2(3,:),3,1));

    % Check random 1000 X1 points on image1
    rand_ = floor(((size(X1,2)) - 1).* rand(1,1000))  + 1;
    X1_rand = X1(:,rand_);
    X2_rand = X2(:,rand_);
    if(store_results == 0)
        figure(2), imshow(image1), title('Image1'); 
        hold on;
        scatter(X1_rand(1,:),X1_rand(2,:), 90, 'b.');
        hold off;
        figure(3), imshow(image2), title('Image2');
        hold on;
        scatter(X2_rand(1,:),X2_rand(2,:), 90, 'rx');
        hold off;
    end

    % Check motion flow information 
    vx = X2(1,:) - X1(1,:);
    vy = X2(2,:) - X1(2,:);
    flow = zeros(size(image1,1), size(image1,2), 2);
    for point=1:size(X1,2)
        row = round(X1(2,point));
        col = round(X1(1,point));
        flow(row, col, 1) = vx(1,point);
        flow(row, col, 2) = vy(1,point);
    end
    flow_image = flowToColor(flow);

    % Intensify flow_color information and save it
    flow_image = intensifyRGB(flow_image, 10);
    figure(5), imshow(imresize(cat(2, im2double(image1), flow_image), 0.5));
    if(store_results == 1)
        % Get current figure
        fig = gcf;
        frame1 = getframe(fig);
        % Write each frame to the file.
        writeVideo(vidObj1,frame1);
    end
    
%     % Check for opticalFlow class (only on MATLAB 2016)
%     flow_opticalFlow = opticalFlow(flow(:,:,1), flow(:,:,2));
%     figure(6), plot(flow_opticalFlow);
end

if(store_results == 1)
    % Close the objects
    close(vidObj1);
end



