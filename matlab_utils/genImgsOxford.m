% To generate RGB images

% Matlab SDK - ../helpers/robotcar-dataset-sdk-1.0/matlab
% Oxford dataset - ../datasets/Oxford_NightTime_Database/

%{
NOTES:-
For '2015-02-03-19-43-11', 
a) No chunks below '04' can be used! Timestamps corresponding to the previous chunks are 
not available in the ins.csv file!
b) No lms_front available!
%}

%% Dependencies here
% Add Matlab SDK
path_robotcar_sdk = './robotcar-dataset-sdk-1.0/matlab';
addpath(genpath(path_robotcar_sdk));
% Set extrinsics dir
extrinsics_dir = '../helpers/robotcar-dataset-sdk-1.0/extrinsics';
% Path to the entire dataset
if(isunix==1)
    path_dataset = '../datasets/Oxford_RobotCar_Dataset/';
    LIME_path    = '~/Dropbox/NightTime_MINDEF_Project/NightTime_Stereo_Vision_Project/LIME_DEMO_CODE/';
else
    path_dataset = 'E:/Aashish_Data/Oxford RobotCar Dataset/Oxford_NightTime_Database/';
    LIME_path    = 'C:/Users/E0021530/Dropbox/NightTime_MINDEF_Project/NightTime_Stereo_Vision_Project/LIME_DEMO_CODE/';
end
% Path to where the results are to be stoed
path_results = '../datasets/Our_Oxford_RobotCar';

%% Read the input file
inputFile    = '../datasets/Our_Oxford_RobotCar_Sheet.txt';
inputFile_id = fopen(inputFile);
inputDataAll = textscan(inputFile_id, '%s %d %s %s %s');
datanameAll  = inputDataAll{1};
chunknoAll   = inputDataAll{2};
lmstypeAll   = inputDataAll{3};
timestamp1All= inputDataAll{4};
timestamp2All= inputDataAll{5};

%% PROVIDE THE DETAILS HERE OF THE DATASET!
% Specific dataset name, and its corresponding chunk
for imgNo = 1:size(datanameAll,1)
    dataname  = datanameAll{imgNo};
    chunk     = ['0', num2str(chunknoAll(imgNo))];
    lms_type  = lmstypeAll{imgNo};
    % Specify the timestamp for im1
    timestampName1 = num2str(timestamp1All{imgNo});
    timestampName2 = num2str(timestamp2All{imgNo});

    %% Now Setup the directories
    % Set image directories
    image1_dir = [path_dataset , dataname, '_stereo_left_', chunk, '/', ...
                  dataname, '/stereo/left'];
    image2_dir = [path_dataset , dataname, '_stereo_left_', chunk, '/', ...
                  dataname, '/stereo/left'];             
    % Set models dir
    models_dir = [path_dataset, 'camera-models'];
    % Read timestamps file for the images
    timestamps1= dlmread([path_dataset , dataname, '_stereo_left_', chunk, '/', ...
                          dataname,'/stereo.timestamps']);
    timestamps2= dlmread([path_dataset , dataname, '_stereo_left_', chunk, '/', ...
                          dataname,'/stereo.timestamps']);


    %% Read the camera models
    [~, ~, ~, ~, ~, img1_LUT] =  ReadCameraModel([path_dataset , dataname, ... 
                                  '_stereo_left_', chunk], models_dir);
    [~, ~, ~, ~, ~, img2_LUT] =  ReadCameraModel([path_dataset , dataname, ...
                                  '_stereo_left_', chunk], models_dir);
                              
    %% Read the images
    % Get the Img0, Img1 idx, and the corresponding timestamp
    image1_idx = getTimestampIdx(timestampName1, timestamps1);
    image2_idx = getTimestampIdx(timestampName2, timestamps2);
    % Get the images
    imageOrig1 = LoadImage(image1_dir, timestamps1(image1_idx), img1_LUT);
    imageOrig2 = LoadImage(image2_dir, timestamps2(image2_idx), img2_LUT);
    % Manually remove the "bonnet" area (roughly 140 rows from the end)
    image1 = imageOrig1(1:end-140, :, :);
    image2 = imageOrig2(1:end-140, :, :);

    %% Save the results
    resize   = 1;
    resizeby = 0.75;
    % Save the two images (additionally re-size if needed)
    result1_dir = [path_results, '/i', num2str(imgNo), '/'];
    result2_dir = [path_results, '/i', num2str(imgNo), '_LIME/'];
    if(~exist(result1_dir, 'dir'))
        mkdir(result1_dir);
    end
    if(~exist(result2_dir, 'dir'))
        mkdir(result2_dir);
    end
    if(resize == 1)
        % Re-size the images
        h_old = size(image1,1);
        w_old = size(image1,2);
        image1 = imresize(image1, resizeby);
        image2 = imresize(image2, resizeby);
    end 
    imwrite(image1, [result1_dir, 'im0_n.png']);
    imwrite(image2, [result1_dir, 'im1_n.png']);
    
    %% Additionally save boosted iamges
    addpath(genpath(LIME_path));
    para.lambda   = 0.15; 
    para.sigma    = 2; 
    para.gamma    = 0.8; 
    para.solver   = 1; 
    para.strategy = 3;
    im0_n_LIME = LIME(imread([result1_dir, 'im0_n.png']), para);
    im1_n_LIME = LIME(imread([result1_dir, 'im1_n.png']), para);
    imwrite(im0_n_LIME, [result2_dir, 'im0_n.png']);
    imwrite(im1_n_LIME, [result2_dir, 'im1_n.png']);
end


