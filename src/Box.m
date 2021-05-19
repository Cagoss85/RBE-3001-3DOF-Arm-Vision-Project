classdef Box
    %Box class for dropping off balls
    %   Boxes are receptacles for the balls. They have a location relative to the robot, and a location that the ball should be dropped from
    
    properties
        color;  %The color ball that should be placed in the box
        location;   %The location of the box relative to the checkerboard/Robot(TBD)
        dropLocation;   %The position relative to the robot that the ball should be dropped from
    end
    
    methods
        
        function obj = Box(color, loc, dropOffLoc)
            %Constructor for a Box
            obj.color = color;
            obj.location = loc;
            obj.dropLocation = dropOffLoc;
        end
    end
end

