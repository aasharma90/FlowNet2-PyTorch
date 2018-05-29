close all;
clear all;

% Set Image dir
image_dir = '../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo/left';
% Set LASER dir
laser_dir = '../../2014-12-10-18-10-50_lms_front_07/2014-12-10-18-10-50/lms_front';
% Set INS file
ins_file = '../../2014-12-10-18-10-50_gps/2014-12-10-18-10-50/gps/ins.csv';
% Set models dir
models_dir = '../../camera-models';
% Set extrinsics dir
extrinsics_dir = '../extrinsics';
% Read timestamps
timestamps = dlmread('../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo.timestamps');

pose_timestamps = [];
origin_timestamp = timestamps(end-2313+(1-1),1);
for i=1:10
    pose_timestamp = timestamps(end-2313+(i-1),1);
    pose_timestamps(i) = pose_timestamp;
end
% Investigate what InterpolatePoses function does
[poses] = InterpolatePoses(ins_file, pose_timestamps, origin_timestamp);
% Observation 1 - 
... It generates Homogeneous Transformation matrices in SE3 format. 
... For the origin timestamp, it generated
... [R      t
... 000     1],    where R is 3x3 I matrix, and t = [0;0;0]
... For the next timestamp, it generates a similar matrix, this time non-zero R and t

% Assuming R and t obtained abvoe can be applied for camera epipolar geometry (Incorrect assumption though !)
R1 = poses{1}(1:3,1:3);
R2 = poses{2}(1:3,1:3);
T1 = poses{1}(1:3, end);
T2 = poses{2}(1:3, end);
% Get intrinsics from ReadCameraModel 
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel(image_dir, models_dir);
K1 = [fx    0    cx
      0     fy   cy
      0     0     1];
K2 = [fx    0    cx
      0     fy   cy
      0     0     1];
% Get [u,v,depth] data from PointCloud Projection
[uv, colours, ~] = ProjectLaserIntoCamera(image_dir, laser_dir, ins_file, models_dir, extrinsics_dir, pose_timestamps(1));

% Load Images
image1 = LoadImage(image_dir, pose_timestamps(1), LUT);
image2 = LoadImage(image_dir, pose_timestamps(2), LUT);

% Check epipolar geometry 
N = size(uv,1);
for i=1:N
        X1(:,i) = [uv(i,1); uv(i,2); 1];
        z(:,i)  = colours(i,1);
end
% Compute the corresponding points in the second image
% Eqn - (K2 * R2' * R1 * inv(K1))*X1   +   (K2 * (T2 - T1)/Z)
% Find corresponding points X2 in image2 
X2(:,:) = ((K2 *  R2' * R1 * inv(K1)) * X1) + repmat((K2 * (T2 - T1)), 1, N)./repmat(z, [3 1]);
X2(:,:) = round(X2(:,:)./repmat(X2(3,:),3,1));

% Check random 100 X1 points on image1
rand_ = floor(((size(X1,2)) - 1).* rand(1,200))  + 1;
X1_rand = X1(:,rand_);
X2_rand = X2(:,rand_);
figure(1), imshow(image1), title('Image1'); 
hold on;
scatter(X1_rand(1,:),X1_rand(2,:), 90, 'b.');
hold off;
% Check corresponding X2 on image2
figure(2), imshow(image2), title('Image2'); 
hold on;
scatter(X2_rand(1,:),X2_rand(2,:), 90, '.');
hold off;

% Check motion flow information 
vx = X2(1,:) - X1(1,:);
vy = X2(2,:) - X1(2,:);
figure(3), imshow(image1),
hold on
quiver(X1(1,:), X1(2,:), vx, vy);
hold off