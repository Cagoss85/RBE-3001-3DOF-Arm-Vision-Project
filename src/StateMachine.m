clc;
clear all;
clear java;
format short;

timer = EventTimer();
sm = StateMachine();

sm.state = State.IDLE;
%% Define Constants

%% Loop
timer.startTimer(5);
try
    while true
        if timer.checkExpired() %If the start up timer has expired
            sm.handleTimer(); %run the handle timer function to move the statemachine
        end
        if sm.ballDetected() == 1 %if a ball is detected
            sm.handleBall(sm.foundBalls(1,:)); %handle the ball
        end
        if sm.aboveBall %if the arm pos is above the targeted ball
            sm.pickupBall(sm.foundBalls(1,:));  %begin the pickup motion
        end
        if sm.hasBall % if the robot has a ball
            sm.depositBall(sm.foundBalls(1,:)); %begin deposit sequence
        end
        if sm.ballDetected() == 0 %if there are no balls left
            sm.handleNoBall(); % move to IDLE and shutdown
            break;
        end
    end
catch exception
    getReport(exception)
    disp('error, exiting while loop');
    sm.robot.shutdown();
end

