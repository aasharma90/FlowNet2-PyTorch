close all;
clear all;
% 
% dbstop BuildPointcloud 149

% Set Image dir
image_dir = '../../2014-12-10-18-10-50_stereo_centre_07/2014-12-10-18-10-50/stereo/centre';
% Set 2-D LASER dir
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
timestamps = dlmread('../../2014-12-10-18-10-50_stereo_centre_07/2014-12-10-18-10-50/stereo.timestamps');

% Generate for 100 frames
chunk = 7;
store_results = 1;
check_frames  = 100;
if(chunk == 7)
    start_frame_index = 1;
elseif(chunk == 1)
    start_frame_index = 3000;
end
if(store_results == 1)
    vidObj1 = VideoWriter('PointCloudProjected.avi');
    vidObj1.FrameRate = 4;
    open(vidObj1);
end

for i=start_frame_index : start_frame_index+check_frames-1
    % Set timestamp
    if(chunk == 7)
    name = [num2str(timestamps(end-2313+(i+490-1),1)), '.png'];
    image_timestamp = timestamps(end-2313+(i+490-1),1);
    elseif(chunk == 1)
        name = [num2str(timestamps(i,1)), '.png'];
        image_timestamp = timestamps(i,1);
    end
    
%     % Check PointCloud
%     BuildPointcloud(laser_dir, ins_file, extrinsics_dir);
%     return;
    
    % Project PointCloud on Image
    [uv, colours, image] = ProjectLaserIntoCamera(image_dir, laser_dir, ins_file, models_dir, extrinsics_dir, image_timestamp);
    figure(1), imshow(image)
    % Display result
    figure(2), imshow(image);
    colormap jet;
    hold on;
    colours = 255.*(colours - min(colours(:)))./(max(colours(:)) - min(colours(:)));
    scatter(uv(:,1),uv(:,2), 90, colours, '.');
%     % Display result
%     figure(3), imshow(image);
%     colormap jet;
%     hold on;
%     reflectance_vals = 255.*(reflectance_vals - min(reflectance_vals(:)))./(max(reflectance_vals(:)) - min(reflectance_vals(:)));
%     scatter(uv(:,1),uv(:,2), 90, reflectance_vals, '.');
    if(store_results == 1)
        % Get current figure
        fig = gcf;
        frame1 = getframe(fig);
        % Write each frame to the file.
        writeVideo(vidObj1,frame1);
    end
end

if(store_results == 1)
    % Close the objects
    close(vidObj1);
end