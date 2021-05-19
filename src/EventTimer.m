classdef EventTimer < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        duration;
        startTime;
        isRunning = false;
    end
    
    methods
        function self = eventTimer(self)
            self.startTime = tic;
            self.duration = 0;
            self.isRunning = false;
        end
        
        function startTimer(self, interval)
            self.duration = interval;
            self.isRunning = true;
            self.startTime = tic;
        end
        
        function stopTimer(self)
            self.isRunning = false;
        end
        
        function output = checkExpired(self)
            if (toc(self.startTime) >= self.duration) && self.isRunning
                output = true;
            else
                output = false;
            end
        end
    end
end

