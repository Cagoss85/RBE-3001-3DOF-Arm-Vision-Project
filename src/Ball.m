classdef Ball < handle
    %Ball Objects that can be found on the field
    %   Balls have a specified color, a centroid position, and an actual position based on either the checkerboard or robot. 
    
    properties
        color;  %String
        
        %Positions of the ball relative to the centroid that the camera sees. 
        centroidPos = [];   %1x2 matrix
        
        %Actual Positions of the ball relative to the checkerboard
        checkerPos = [];    %1x2 matrix
        
        %Actual Positions of the ball relative to the base of the robot. 
        %These will be the most important points. 
        robotPos = [];      %1x2 matrix
    end
    
    methods
        %Create a ball object with a determined color
        function self = Ball(col)
            self.color = col;
            self.centroidPos = [];
            self.checkerPos = [];
            self.robotPos = [];
        end
        
        %Function takes in a 1x2 matrix corresponding to a centroid position calculated by the camera
        %Function sets the balls centroid parameters to the input values
        function self = setCentroidPos(self, pos)
            %Set a balls centroid position as seen by the camera.
            self.centroidPos(1) = pos(1);
            self.centroidPos(2) = pos(2);
        end
        
        %Function takes in a 1x2 matrix corresponding to a balls actual position relative to the checkerboard
        %Function sets the balls checkerPos parameters to the input values
        function setCheckerPos(self, pos)
            %Set a balls position relative to the checkerboard.
            self.checkerPos(1) = pos(1);
            self.checkerPos(2) = pos(2);
        end
        
        %Function takes in a 1x2 matrix corresponding to a balls actual position relative to the robot
        %Function sets the balls robotPos parameters to the input values
        function setRobotPos(self, pos)
            %Set a balls position relative to the robot. 
            self.robotPos(1) = pos(1);
            self.robotPos(2) = pos(2);
        end
        
        %Function sets input positions to the ball parameters, and calculates where the ball actually is.
        %Function requires that a ball have stored parameters for where the camera thinks it is. 
        %Function also sets the balls actual position to the calculated positions
        %Function outputs the actual positions of the ball relartive to the robot. 
        function output = calcActualPos(self)
            %Calculate a balls position based on the centroid position
            hCam = 141;     %height of the camera, 121mm stand + 20mm to lens
            r = 10.8;       %radius of a ball, DOES take flat bottom into account
            xOffset = 99;   %Translations from the checkerboard origin to the camera base.
            yOffset = 151;
            
            %Get params from the object.
            xPos = self.centroidPos(1);
            yPos = self.centroidPos(2);
            
            %Calculate where the ball is relative to the camera base. 
            xCam = xPos - xOffset;
            yCam = yOffset - yPos;
            
            p = sqrt(xCam^2 + yCam^2);  %Horizontal distance between the camera and the centroid
            theta = atand(hCam/p);      %Angle between the balls centroid and the camera.
            a = r/tand(theta);          %Distance from the centroid to the actual base along "p"
            
            beta = atan2d(xCam,yCam);   %Angle of the ball relative to the camera
            
            xBallCam = (p - a)*sind(beta);  %The actual x and y positions of the ball relative to the camera
            yBallCam = (p - a)*cosd(beta);
            
            xBallChecker = xBallCam + xOffset; %Actual ball locations relative to the checkerboard
            yBallChecker = yOffset - yBallCam;
            
            actualPosChecker = [xBallChecker yBallChecker]; %Set the balls params for the location relative to the checkerboard
            self.setCheckerPos(actualPosChecker);
            
            tBase2Checker = [0 1 0 50; 1 0 0 -100; 0 0 -1 0; 0 0 0 1];  %Transformation matrix from the base to the checkerboard
            pChecker = [xBallChecker; yBallChecker; 0; 1];              %P matrix
            actualPosRobot = tBase2Checker * pChecker;                  
            
            self.setRobotPos(actualPosRobot);   %Set the balls parameters to the actual positions
            
            output = [self.robotPos(1) self.robotPos(2)];   
        end
    end
end

