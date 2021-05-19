classdef Camera
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;  
        DEBUG_BALLDETECTION = false;
        
        % Image Processing Variables
        
        % Colors
        
        % Properties
        params;
        cam;
        cam_pose;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(); % Get camera object
            self.params = self.loadIntrinsics(); % Run Calibration Function
            
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
        
        function [ball_data] = findBalls(self)
            
            %taking a picture and undistorting it
            [img, newParams] = self.undistortSnapshot();
            %figure(1);
            %imshow(img);
            
            %setting up coordinates for the workspace mask
            x1 = [466.25 1427.75 1228.25 682.25];
            y1 = [760.25 772.25 377.75 371.75];
            
            
            %making the new image by mulitplying the original image by the
            %mask
            mask = roipoly(img, x1, y1);
            img = im2double(img);
            img = mask.*img;
            %imshow(img);
            
            %applying the hsv filters to the masked workspace image
            lime_green = lime_green_Mask(img);
            red_orange = red_orange_Mask(img);
            pink = pink_Mask(img);
            yellow = yellow_Mask(img);
            
            %getting rid of noise from the hsv output 
            noise_reduction_green = medfilt2(lime_green, [20 20]);
            noise_reduction_red = medfilt2(red_orange, [20 20]);
            noise_reduction_pink = medfilt2(pink, [20 20]);
            noise_reduction_yellow = medfilt2(yellow, [20 20]);
            
            %finding the center points of the balls in pixel space 
            region_green = regionprops(noise_reduction_green, 'centroid');
            region_red = regionprops(noise_reduction_red, 'centroid');
            region_pink = regionprops(noise_reduction_pink, 'centroid');
            region_yellow = regionprops(noise_reduction_yellow, 'centroid');
            
%             %displaying the noise reduction outputs 
%             figure(3)
%             subplot(2,2,1)
%             imshow(noise_reduction_green);
%             title('Green');
%             
%             subplot(2,2,2)
%             imshow(noise_reduction_red);
%             title('Red');
%             
%             subplot(2,2,3)
%             imshow(noise_reduction_pink);
%             title('Pink');
%             
%             subplot(2,2,4)
%             imshow(noise_reduction_yellow);
%             title('Yellow');
            
            %making a matrix to hold all the ball data while also
            %converting the pixel space coords to checkerboard coords
            if isempty(region_green)
                green = [];
            else
                gPos = self.cam2Base([region_green.Centroid(1) region_green.Centroid(2)],false);
                %gPos = [gPos(1); gPos(2)];
                %gPos = self.calcActualPos(gPos);
                green = [1 gPos(1) gPos(2)];
            end
            
            if isempty(region_red)
                red = [];
            else
                rPos = self.cam2Base([region_red.Centroid(1) region_red.Centroid(2)],false);
                %rPos = [rPos(1); rPos(2)];
                %rPos = self.calcActualPos(rPos);
                red = [2 rPos(1) rPos(2)];
            end
            
            if isempty(region_pink)
                pink = [];
            else
                pPos = self.cam2Base([region_pink.Centroid(1) region_pink.Centroid(2)],false);
                %pPos = [pPos(1); pPos(2)];
                %pPos = self.calcActualPos(pPos);
                pink = [3 pPos(1) pPos(2)];
            end
            
            if isempty(region_yellow)
                yellow = [];
            else
                yPos = self.cam2Base([region_yellow.Centroid(1) region_yellow.Centroid(2)],false);
                % yPos = [yPos(1); yPos(2)];
                %yPos = self.calcActualPos(yPos);
                yellow = [4 yPos(1) yPos(2)];
            end
            
            ball_data = cat(1, green, red, pink, yellow);
            
%             figure(1)
%             imshow(img)
%             hold on
%             if isempty(region_green) == 0
%                 plot(region_green.Centroid(1), region_green.Centroid(2), 'b*')
%             end
%             if isempty(region_red) == 0
%                 plot(region_red.Centroid(1), region_red.Centroid(2), 'b*')
%             end
%             if isempty(region_pink) == 0
%                 plot(region_pink.Centroid(1), region_pink.Centroid(2), 'b*')
%             end
%             if isempty(region_yellow) == 0
%                 plot(region_yellow.Centroid(1), region_yellow.Centroid(2), 'b*')
%             end
%             hold off
        end
        
        %Function outputs the actual positions of the ball relartive to the robot. 
        function output = calcActualPos(self,posCentroid)
            %Calculate a balls position based on the centroid position
            hCam = 141;     %height of the camera, 121mm stand + 20mm to lens 
            r = 10.8;       %radius of a ball, DOES take flat bottom into account
            xOffset = 99;   %Translations from the checkerboard origin to the camera base.
            yOffset = 151;
            
            %Get params from the object.
            xRobot = posCentroid(1);
            yRobot = posCentroid(2);
            
            %Calculate where the ball is relative to the camera base. 
            xCam = (yRobot + 100) - xOffset;
            yCam = yOffset - (xRobot - 50);
            
            p = sqrt(xCam^2 + yCam^2);  %Horizontal distance between the camera and the centroid
            theta = atand(hCam/p);      %Angle between the balls centroid and the camera.
            a = r/tand(theta);          %Distance from the centroid to the actual base along "p"
            
            beta = atan2d(xCam,yCam);   %Angle of the ball relative to the camera
            
            xBallCam = (p - a)*sind(beta);  %The actual x and y positions of the ball relative to the camera
            yBallCam = (p - a)*cosd(beta);
            
            xBallRobot = (yOffset - yBallCam) + 50;%Actual ball locations relative to the robot
            yBallRobot = (xBallCam + xOffset) - 100;                
            
            r = sqrt(xBallRobot^2 + yBallRobot^2);
            r = r+3; %Magic number for calibration
            thetaPolar = atan2d(yBallRobot, xBallRobot);
            thetaPickup = thetaPolar - 7;
            
            xPickup = r*cosd(thetaPickup);
            yPickup = r*sind(thetaPickup);
            
            
            output = [xPickup yPickup];   
        end
        
        function params = loadIntrinsics(self)
            % CALIBRATE Calibration function
            % Function loads stored camera parameters from a specified file.
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                params = load('camParamFish'); % This corresponds to your calibration file
                disp("Camera calibration complete! Place Balls Now!");
            catch exception
                getReport(exception);
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        function pose = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.cameraParams.Intrinsics);
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Adjust imagePoints so they are in the same frame of
            % reference as original image
            %imagePoints = imagePoints + undistortFisheyePoints(imagePoints, self.params.Intrinsics);
            % 5. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.cameraParams.WorldPoints, newIs);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                axesPoints = worldToImage(self.params, R, t, [0 0 0; 0 50 0; 50 0 0]);
                
                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end     
        end
        
        %Function returns an undistorted view of the checkerboard
        function [a,b] = undistortSnapshot(self)
            img = snapshot(self.cam);
            [a, b] = undistortFisheyeImage(img, self.params.cameraParams.Intrinsics);
        end
        
        function img = snapshot(self)
            %function takes a snapshot with the camera, returns a distorted image. 
            img = snapshot(self.cam);
        end
            
        %Function inputs a pixel coordinate
        %Function outputs that coordinate in terms of the robot base
        function pos = cam2Base(self, coord, isDistorted)
            %Function converts input pixel coordinate to a coordinate based on the robot. Input pixel can be interms of a distorted or undistorted image, set by parameter. 
            if isDistorted == false
                [img, undistParams] = self.undistortSnapshot();
                parameters = undistParams;
            else
                parameters = self.params.cameraParams.Intrinsics;
            end
            worldPoints = pointsToWorld(parameters, self.cam_pose(1:3,1:3), self.cam_pose(1:3,4), coord);
            tBase2Checker = [0 1 0 50; 1 0 0 -100; 0 0 -1 0; 0 0 0 1];
            pChecker = [worldPoints(1); worldPoints(2); 0; 1];
            pBase = tBase2Checker * pChecker;
            pos = pBase;
        end
    end
end

