classdef StateMachine < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        state;
        robot;
        hasBall = false;
        aboveBall = false;
        fakeBall = true;
        camera;
        ballDetectedFlag = false;
        foundBalls = [];
    end
    
    methods
        function self = StateMachine()
            vid = hex2dec('16c0');
            pid = hex2dec('0486');
            
            javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
            import edu.wpi.SimplePacketComs.*;
            import edu.wpi.SimplePacketComs.device.*;
            import edu.wpi.SimplePacketComs.phy.*;
            import java.util.*;
            import org.hid4java.*;
            version -java;
            
            myHIDSimplePacketComs=HIDfactory.get();
            myHIDSimplePacketComs.setPid(pid);
            myHIDSimplePacketComs.setVid(vid);
            myHIDSimplePacketComs.connect();
            
            self.robot = Robot(myHIDSimplePacketComs);
            self.state = State.IDLE;
            self.camera = Camera();
            self.camera.cam_pose = self.camera.getCameraPose();
        end
        
       %Function that sends the robot to zero position
       %switches the states once the start up timer has expired
        function handleTimer(self)
            %Function responds when a timer is expired.
            switch(self.state)
                case State.IDLE
                    disp('Timer is Done! Getting Ready To Move');
                    self.robot.servo_jp([0 0 0]);
                    pause(3);
                    self.state = State.SCAN;
            end
        end
        
        function output = ballDetected(self)
            status = [];
            switch (self.state)
                case State.SCAN
                    var = self.camera.findBalls();
                    disp(var);
                    self.foundBalls = var;
                    if isempty(self.foundBalls)
                        disp('No Balls Found on Board!');
                        status = 0;
                    else
                        disp('I Found a Ball!');
                        status = 1;
                        %self.state = State.MOVE;
                    end
            end
            output = status;
        end
        
        %Function that handles when a ball is found. Moves to above the ball and
        %marks the correct flags to continue the statemachine
        function handleBall(self, ball)
            switch(self.state)
                case State.IDLE
                case State.SCAN %SHOULD BE CHANGED TO MOVE ONCE CAMERA WORKS
                    disp('Moving to Above the Ball!');
                    %pause(2);
                    self.robot.moveAbovePos([ball(2), ball(3), 12.5],150);
                    pause(.5);
                    self.aboveBall = true;
                    self.fakeBall = false;
                    self.state = State.MOVE;
            end
        end
        
        %Function that handles the pickupBall movement
        % moves from above the ball slowly down to the ball position
        % Once in pickup position, it changes states, and closes the gripper
        % It marks the correct flags to continue the state
        function pickupBall(self, ball)
            switch(self.state)
                case(State.IDLE)
                case(State.SCAN)
                case(State.MOVE)
                    disp('Moving to Pickup!');
                    self.robot.moveToPickupPos([ball(2),ball(3), 16], 150);
                    pause(1);
                    self.state = State.GRASP;
                    
                case(State.GRASP)
                    disp('Time to Grab!');
                    self.robot.moveGripper(90);
                    pause(1);
                    disp('I have the Ball!');
                    self.hasBall = true;
                    self.state = State.MOVE;
                    self.aboveBall = false;
            end
        end
        
        % This function raises the arm in the arm from the position it picked up the ball in 
        % This makes it so that the arm will not knock around other balls
        % From there the robot will bring it to the correct drop bin based on the color ball it
        % is minipulating. Once over the bin, it will switch states, and drop the ball, setting the 
        % correct flags. 
        function depositBall(self,ball)
            color = ball(1); %Determine the color of the ball
            switch(self.state)
                case(State.IDLE)
                case(State.SCAN)
                case(State.MOVE)
                    disp('Moving to Deposit the Ball');
                    self.robot.raiseArm([ball(2),ball(3), 12.5],100);
                    pause(1);
                    switch (color)
                        case(1) %green
                            self.robot.moveToDropPos([155,125, 100], 150);
                        case(2) %pink
                            self.robot.moveToDropPos([90, 160, 100], 150);
                        case(3) %orange
                            self.robot.moveToDropPos([45, 160, 100], 150);
                        case(4) %yellow
                            self.robot.moveToDropPos([172, 95, 100], 150);
                    end
                    pause(1);
                    self.state = State.GRASP;
                case(State.GRASP)
                    disp('Dropping the Ball');
                    self.robot.moveGripper(180);
                    %pause(2);
                    self.hasBall = false;
                    self.ballDetectedFlag = true;
                    self.state = State.SCAN;
                    self.robot.ik_3001_final_num([100 0 195],300);
                    pause(1);
            end
        end
        
        %This function switches the robot to IDLE and shuts down the robot if
        %there are no more balls on the field to sort. 
        function handleNoBall(self)
            switch(self.state)
                case(State.IDLE)
                case(State.SCAN)
                    disp('Bed Time');
                    self.robot.ik_3001_final_num([100 0 195],150);
                    pause(2);
                    self.robot.shutdown();
                    self.state = State.IDLE;
                case(State.MOVE)
                case(State.GRASP)
            end
        end
        
        
        
    end
end