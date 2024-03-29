% Auto-generated by cameraCalibrator app on 08-Mar-2021
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image1.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image2.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image5.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image6.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image7.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image8.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image9.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image10.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image11.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image12.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image13.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image16.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image17.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image20.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image21.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image22.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image23.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image26.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image28.png',...
    '/home/amber/RBE3001Code09/camera_calibration/Amber_calib_pics/Image29.png',...
    };
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 25;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera using fisheye parameters
[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...
    [mrows, ncols], ...
    'EstimateAlignment', false, ...
    'WorldUnits', 'millimeters');

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);
save('AmbersSavedParams', 'cameraParams');
% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
