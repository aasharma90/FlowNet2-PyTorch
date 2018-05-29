% Read timestamps
timestamps1 = dlmread('../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo.timestamps');
% Generate LUT model
[ ~, ~, ~, ~, ~, LUT1] =  ReadCameraModel('../../2014-12-10-18-10-50_stereo_left_07', '../../camera-models');

% Read timestamps
timestamps2 = dlmread('../../2014-12-10-18-10-50_stereo_right_07/2014-12-10-18-10-50/stereo.timestamps');
% Generate LUT model
[ ~, ~, ~, ~, ~, LUT2] =  ReadCameraModel('../../2014-12-10-18-10-50_stereo_right_07', '../../camera-models');

% Generate for 100 frames
check_frames = 100;
vidObj1 = VideoWriter('ImageSequence.avi');
vidObj1.FrameRate = 4;
open(vidObj1);

for i=1:check_frames
    % Set timestamp
    image1_timestamp = timestamps1(end-2313+(i-1),1);
    image2_timestamp = timestamps2(end-2313+(i-1),1);
    % Load Images
    image1 = LoadImage('../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo/left', image1_timestamp, LUT1);
    image2 = LoadImage('../../2014-12-10-18-10-50_stereo_right_07/2014-12-10-18-10-50/stereo/right', image2_timestamp, LUT2);
    % Save Images
    name1 = [num2str(timestamps1(end-2313+(i-1),1)), '.png'];
    name2 = [num2str(timestamps2(end-2313+(i-1),1)), '.png'];
    imwrite(image1,['../../2014-12-10-18-10-50_stereo_left_07/2014-12-10-18-10-50/stereo/left_rectified/', name1]);
    imwrite(image2,['../../2014-12-10-18-10-50_stereo_right_07/2014-12-10-18-10-50/stereo/right_rectified/', name2]);
    % Get Combined Image
    image_new = imadd(image1, image2);
    frame1 = im2frame(image_new);
    % Write each frame to the file.
    writeVideo(vidObj1,frame1);
end

% Close the objects
close(vidObj1);