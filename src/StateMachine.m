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
    end
catch exception
    getReport(exception)
    disp('error, exiting while loop');
    sm.robot.shutdown();
end

