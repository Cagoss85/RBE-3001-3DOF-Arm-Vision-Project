classdef Traj_Planner
    %Traj Planner class take in an nx3 array as input
    %Each row represents the x,y,z coordinates of the tip over n viapoints
    properties
        Data;   %The xyz coordinates for the end effector
        xData;  %The x coordinates for the end effector
        yData;  %The y coordinates for the end effector
        zData;  %The z coordinates for the end effector
        
    end
    
    methods
        %Object Constructor
        function obj = Traj_Planner(input)
            obj.Data = input;
            obj.xData = input(:,1);
            obj.yData = input(:,2);
            obj.zData = input(:,3);
        end
        
        %Function solves for a cubic trajectory between 2 points, it **SHOULD** work in joint and task space
        function output = cubic_traj(self,t0,tf,v0,vf,q0,qf)
            %Set equation variables
            syms a0 a1 a2 a3;
            vars = [a0 a1 a2 a3];
            
            %Set Equations
            eqns = [a0 + a1*(t0)    + a2*(t0^2)     + a3*(t0^3)  == q0,
                    a1 + 2*a2*(t0)  + 3*a3*(t0^2)                == v0,
                    a0 + a1*(tf)    + a2*(tf^2)     + a3*(tf^3)  == qf,
                    a1 + 2*a2*(tf)  + 3*a3*(tf^2)                == vf];
            
            [A,B] = equationsToMatrix(eqns,vars);           %Create the trajectory matrix
            
            traj = [A,B];                                   %Set to a variable
            %disp('Trajectory Matrix is');
            %disp(traj);
            
            sol = linsolve(A,B);                            %Solve the matrix for a0-3
            %disp('Solution Set is');
            %disp(sol);
            output = sol;
        end
        function output = quintic_traj(self, t0,tf,v0,vf,q0, qf)
            %set variables
            syms a0 a1 a2 a3 a4 a5;
            vars = [a0 a1 a2 a3 a4 a5];
            alpha0 = 0;
            alphaf = 0;
            %set all of the equations into a matrix
            eqns = [a0+a1*t0+a2*(t0^2)+a3*(t0^3)+a4*(t0^4)+a5*(t0^5)==q0,...
                    0+a1+2*a2*t0+3*a3*(t0^2)+4*a4*(t0^3)+5*a5*(t0^4)==v0,...
                    0+0+2*a2+6*a3*t0+12*a4*(t0^2)+20*a5*(t0^3)==alpha0,...
                    a0+a1*tf+a2*(tf^2)+a3*(tf^3)+a4*(tf^4)+a5*(tf^5)==qf,...
                    0+a1+2*a2*tf+3*a3*(tf^2)+4*a4*(tf^3)+5*a5*(tf^4)==vf,...
                    0+0+2*a2+6*a3*tf+12*a4*(tf^2)+20*a5*(tf^3)==alphaf];
            
            [A,B] = equationsToMatrix(eqns, vars); %created the trajectory matrix
            
            traj=[A,B];                               %Set to a variable
            %disp('Trajectory Matrix is');
            %disp(traj);
            
            sol = linsolve(A,B);                            %Solve the matrix for a0-5
            %disp('Solution Set is');
            %disp(sol);
            output = sol;
            
            
            
        end
        
        function output = linear_traj(self, p1, p2)
            
            %generate cubic trajectory between the point given points (in
            %the form of a 4x1 array 
            
            %generating the trajectories for the axes of motion
            tf = 3;
            
            traX = self.cubic_traj(0, tf, 0, 0, p1(1), p2(1))
            traY = self.cubic_traj(0, tf, 0, 0, p1(2), p2(2))
            traZ = self.cubic_traj(0, tf, 0, 0, p1(3), p2(3))
            
            taskSpace = [p1];
            i = 0;
            t = 0;
            increment = 15;
            
            %generating entire task space points 
            while i < increment 
                t = t + tf/increment;
                placeHolder = [traX(1)+traX(2)*t+traX(3)*t^2+traX(4)*t^3 traY(1)+traY(2)*t+traY(3)*t^2+traY(4)*t^3 traZ(1)+traZ(2)*t+traZ(3)*t^2+traZ(4)*t^3]; 
                taskSpace = cat(1, taskSpace, placeHolder);
                i = i + 1;
            end
            
            output = taskSpace; 
            
        end
    end
end

