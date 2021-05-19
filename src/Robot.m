classdef Robot < handle
    properties
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol 
        goal
    end
    methods
    %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
        %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
            packet.goal = [-90.0, 86.14, 33.71];
        end
        
        %Perform a command cycle. This function will take in a command ID and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('jaserva.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret =   packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        %Function reads packet sent from robot
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try
 
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    ret =   packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
                     
        %Function writes a packet to the robot using a specified protocol
        function  write(packet, idOfCommand, values)    
                try
                    packet.goal(1) = values(3);
                    packet.goal(2) = values(4);
                    packet.goal(3) = values(5);
                    
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);
                    
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end       
        end
        
        %Function moves the gripper using a 0-180 value. Function uses bytes instead of floats
        function  moveGripper(packet, values)
            try
                ds = javaArray('java.lang.Byte',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Byte(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(1962);
                packet.myHIDSimplePacketComs.writeBytes(intid,  ds,packet.pol);
                
            catch exception
                getReport(exception)
                disp('Command error, unable to move gripper');
            end
        end
        
        %Function returns current position or velocity data of the robot using 2 input boolean values
        function dataKinematic = measured_js(packet, GETPOS, GETVEL)
                    velReturnPacket = packet.read(1822);       %read the return packet with protocol 1822
                    posReturnPacket = packet.read(1910);       %read the return packet with protocol 1910
                    dataKinematic = zeros(2,3);
                    if(GETPOS)
                        dataKinematic(1,1) = posReturnPacket(3);   %assign the position values to the output matrix
                        dataKinematic(1,2) = posReturnPacket(5);
                        dataKinematic(1,3) = posReturnPacket(7);
                    end
                    if(GETVEL)
                        dataKinematic(2,1) = velReturnPacket(3);   %assign the velocity values to the output matrix
                        dataKinematic(2,2) = velReturnPacket(6);
                        dataKinematic(2,3) = velReturnPacket(9);
                    end
        end
        
        %Function returns the stored goal for the robot
        function RETgoal = goal_js(packet)      %Return the stored goal for the robot
                    RETgoal = packet.goal;
        end
        
        %Take in a 1x3 matrix and the time wanted to interpolate over in ms 
        function interpolate_jp(self, jointValues, time) 
            packet = zeros(15, 1, 'single');
            packet(1)= time;
            packet(3)=jointValues(1);
            packet(4)=jointValues(2);
            packet(5)= jointValues(3);
            self.write(1848, packet);
        end
        
        %Function returns current joint setpoint
        function currentJointSetPt = setpoint_js(self) 
            currentJointSetPt = zeros(1,3);                 %initialize the return matrix
            try
                SERVER_ID_READ =1910;                       % ID of the read packet
                readArray = self.read(SERVER_ID_READ);      %read the robot values
                currentJointSetPt(1) = readArray(2);        %assign the packet values to the output matrix
                currentJointSetPt(2) = readArray(4);
                currentJointSetPt(3) = readArray(6);
            
            catch exception
                getReport(exception)
                disp('failed setpoint_js');
            end
        end
 
        % Function takes in a 1x3 array of commanded joint values in degrees to be sent directly to the actuators and bypasses interpolation
        function servo_jp(self, jointVals)
            try
                SERV_ID = 1848;
                SERV_ID_READ = 1910;
                DEBUG = false;
                
                %set packet values
                packet = zeros(15,1,'single');      %initialize the packet
                packet(1) = 0;                      %disable interpolation
                packet(2) = 0;                      %disable interpolation
                packet(3) = jointVals(1);           %assign the input values to the packet
                packet(4) = jointVals(2);
                packet(5) = jointVals(3);
              
                self.write(SERV_ID,packet);
                returnPacket = self.read(SERV_ID_READ);     %read the return packet if necessary
                
                %method debugging
                if DEBUG
                    disp('Sent Packet:')        %display sent packet
                    disp(packet);
                    disp('Recieved Packet');    %diapley returned packet
                    disp(returnPacket);
                end
            catch exception
                getReport(exception)
                disp('Command error, failed on servo_js')
            end          
        end
        
        %function takes in a 1 x 4 array corrosponding to a row of the DH parameter table for the given link. It generates the associated
        %intermediate transformation and returns the corrosponding 4 x4 homogeneous transformation matrix
        function output = dh2mat(self, matrix) %(theta, d, a, alpha)
            theta = matrix(1,1);
            d = matrix(1,2);
            a = matrix (1,3);
            alpha = matrix(1,4);
            output = [  cos(theta)  -sin(theta)*cos(alpha)    sin(theta)*sin(alpha)   a*cos(theta);
                        sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)   a*sin(theta);
                                 0              sin(alpha)               cos(alpha)              d;
                                 0                       0                        0              1];
        end
        
        %function takes in an nx4 array corresponding to the n rows of the DH table. It generates a symbolic 4x4 trans matrix for the full transformation.
        function output = dh2fk(self, table)
            trans = [];
            for k = 1:size(table,1)                      %iterate over every row of the matrix
               trans{k} = self.dh2mat(table(k,:));       %generate matrix for each row, intex it with a name
               %disp(trans{k});
            end
            
            prev = eye(4);  
            
            for i=1:size(table,1)
                output{i}=prev*trans{i};            %determine the transformation from the base to the ith joint
                prev = output{i};                   %set last transformation for use in the next calculation
            end                    
        end
        
        %function that returns the 4x4 homogeneous transformation matrix for the tip based on the position of the vector. 
        function output = fk3001(self, position, tNum)
            L0 = 55;                                    %set the inputs of lengths
            L1 = 40;
            L2 = 100;
            L3 = 100;
            q1 = deg2rad(position(1,1));                % set the positions equal to the input matrix
            q2 = deg2rad(position(2,1));
            q3 = deg2rad(position(3,1));
            a1 = deg2rad(-90);
            
            switch tNum
                case 1
                    %disp('Transformation from base to frame 1');
                    output =    [ cos(q1), -cos(a1)*sin(q1),  sin(a1)*sin(q1),       0;
                                  sin(q1),  cos(a1)*cos(q1), -sin(a1)*cos(q1),       0;
                                        0,          sin(a1),          cos(a1), L0 + L1;
                                        0,                0,                0,       1]; 
                case 2
                    %disp('Transformation from base to frame 2');
                    output =     [cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2), - cos(q1)*sin(q2 - pi/2) - cos(a1)*cos(q2 - pi/2)*sin(q1),  sin(a1)*sin(q1), L2*cos(q1)*cos(q2 - pi/2) - L2*cos(a1)*sin(q1)*sin(q2 - pi/2);
                                  cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2),   cos(a1)*cos(q1)*cos(q2 - pi/2) - sin(q1)*sin(q2 - pi/2), -sin(a1)*cos(q1), L2*cos(q2 - pi/2)*sin(q1) + L2*cos(a1)*cos(q1)*sin(q2 - pi/2);
                                                                   sin(a1)*sin(q2 - pi/2),                                    sin(a1)*cos(q2 - pi/2),          cos(a1),                           L0 + L1 + L2*sin(a1)*sin(q2 - pi/2);
                                                                                        0,                                                         0,                0,                                                             1];
                otherwise
                    %disp('Transformation from base to frame 3');
                    output =    [ cos(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2)) - sin(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)), - cos(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) - sin(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2)),  sin(a1)*sin(q1), L3*cos(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2)) - L3*sin(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) + L2*cos(q1)*cos(q2 - pi/2) - L2*cos(a1)*sin(q1)*sin(q2 - pi/2);
                                  cos(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)) - sin(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)), - cos(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) - sin(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)), -sin(a1)*cos(q1), L3*cos(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)) - L3*sin(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) + L2*cos(q2 - pi/2)*sin(q1) + L2*cos(a1)*cos(q1)*sin(q2 - pi/2);
                                                                                                        sin(a1)*cos(q2 - pi/2)*sin(q3 + pi/2) + sin(a1)*cos(q3 + pi/2)*sin(q2 - pi/2),                                                                         sin(a1)*cos(q2 - pi/2)*cos(q3 + pi/2) - sin(a1)*sin(q2 - pi/2)*sin(q3 + pi/2),          cos(a1),                                                                                                 L0 + L1 + L2*sin(a1)*sin(q2 - pi/2) + L3*sin(a1)*cos(q2 - pi/2)*sin(q3 + pi/2) + L3*sin(a1)*cos(q3 + pi/2)*sin(q2 - pi/2);
                                                                                                                                                                                    0,                                                                                                                                                     0,                0,                                                                                                                                                                                                                         1];
            end
        end
        
        %Function takes data from measured_js and returns a 4x4 transformation matrix based on current position in deg
        function transMatrix = measured_cp(self)
                transMatrix = [];
                currPos = self.measured_js(1,0);    %Get current data using measured_js
                pos = zeros(3,1);
                pos(1,1) = currPos(1,1);
                pos(2,1) = currPos(1,2);
                pos(3,1) = currPos(1,3);
                transMatrix = self.fk3001(pos,3);
        end
        
        %Function takes data from setpoint_js and returns the transformation matrix for the setpoint
        function transMatrix = setpoint_cp(self) 
            transMatrix = [];
            currPos = self.setpoint_js();       %get current pos from the setpoint_js
            pos(1,1) = deg2rad(currPos(1,1));   %set the position of setpoint js equal to the position to input
            pos(2,1) = deg2rad(currPos(1,2));
            pos(3,1) = deg2rad(currPos(1,3));
            transMatrix = self.fk3001(pos,3);     %tranpose matrix with setpoint position
        end
        
        %Function takes data from goal_js and returns the transformation matrix for the goal
        function transMatrix = goal_cp(self) 
            transMatrix = [];
            currPos = self.goal_js();           %get current pos from the goal_js function
            pos(1,1) = deg2rad(currPos(1,1));   %set the position of goal js equal to the position to input
            pos(2,1) = deg2rad(currPos(1,2));
            pos(3,1) = deg2rad(currPos(1,3));
            transMatrix = self.fk3001(pos,3);     %tranpose matrix with goal position
        end
        
        %Function takes in 3x1 array and draws the robot on a plot, showing joints and axes
        function plot_arm(self, angles)
            pos = zeros(3,1);
            pos(1,1) = angles(1,1);
            pos(2,1) = angles(2,1);
            pos(3,1) = angles(3,1);
            
            T01 = self.fk3001(pos,1);
            %disp(T01);
            T02 = self.fk3001(pos,2);
            %disp(T02);
            T03 = self.fk3001(pos,3);
            %disp(T03);
            
            Qx = [0 T01(1,4) T02(1,4) T03(1,4)];
            Qy = [0 T01(2,4) T02(2,4) T03(2,4)];
            Qz = [0 T01(3,4) T02(3,4) T03(3,4)];
            Q = [Qx; Qy; Qz];
            
            hold on;
            plot3(Q(1,:), Q(2,:), Q(3,:));
            grid on;
            axis equal;
            title('Robot Position');
            
            xlabel('xaxis');
            xlim([-18 230]);
            
            ylabel('yaxis');
            ylim([-140 180]);
            
            zlabel('zaxis');
            %zlim([0 300]);
           
            %Frame Creation
            %Each frame has a specified origin, and a point to draw a line to
            s = 20; %Scale for axes
            l = 3; %lineweight for axes
            
            %Frame 1 X Axis
            p0 = [0 0 0];
            p1 = [s 0 0];
            pts = [p0;p1];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'r', 'LineWidth', l);
            
            %Frame 1 Y Axis
            p0 = [0 0 0];
            p1 = [0 s 0];
            pts = [p0;p1];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'g', 'LineWidth', l);
            
            %Frame 1 Z Axis
            p0 = [0 0 0];
            p1 = [0 0 s];
            pts = [p0;p1];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'b', 'LineWidth', l);
            
            %Frame 2 X Axis
            Po2 = [T01(1,4) T01(2,4) T01(3,4)];
            P2 = [T01(1,1)*s T01(2,1)*s T01(3,4)];
            pts = [Po2; P2];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'r', 'LineWidth', l);
            
            %Frame 2 Y Axis
            P2 = [T01(1,2)*s T01(2,2)*s 75];
            pts = [Po2; P2];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'g', 'LineWidth', l);
            
            %Frame 2 Z Axis
            P2 = [T01(1,3)*s T01(2,3)*s T01(3,4)];
            pts = [Po2; P2];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'b', 'LineWidth', l);
            
            %Frame 3 X Axis
            Po3 = [T02(1,4) T02(2,4) T02(3,4)];
            P3 = [T02(1,4)+s*T02(1,1) T02(2,4)+s*T02(2,1) T02(3,4)+s*T02(3,1)];
            pts = [Po3; P3];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'r', 'LineWidth', l);
            
            %Frame 3 Y Axis
            Po3 = [T02(1,4) T02(2,4) T02(3,4)];
            P3 = [T02(1,4)+s*T02(1,2) T02(2,4)+s*T02(2,2) T02(3,4)+s*T02(3,2)];
            pts = [Po3; P3];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'g', 'LineWidth', l);
            
            %Frame 3 Z Axis
            Po3 = [T02(1,4) T02(2,4) T02(3,4)];
            P3 = [T02(1,4)+s*T02(1,3) T02(2,4)+s*T02(2,3) T02(3,4)+s*T02(3,3)];
            pts = [Po3; P3];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'b', 'LineWidth', l);
            
            %Frame 4 X Axis
            Po4 = [T03(1,4) T03(2,4) T03(3,4)];
            P4 = [T03(1,4)+s*T03(1,1) T03(2,4)+s*T03(2,1) T03(3,4)+s*T03(3,1)];
            pts = [Po4; P4];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'r', 'LineWidth', l);
            
            %Frame 4 Y Axis
            P4 = [T03(1,4)+s*T03(1,2) T03(2,4)+s*T03(2,2) T03(3,4)+s*T03(3,2)];
            pts = [Po4; P4];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'g', 'LineWidth', l);
            
            %Frame 4 Z Axis
            P4 = [T03(1,4)+s*T03(1,3) T03(2,4)+s*T03(2,3) T03(3,4)+s*T03(3,3)];
            pts = [Po4; P4];
            plot3(pts(:,1), pts(:,2), pts(:,3), 'color', 'b', 'LineWidth', l);     
        end
        
        %Function plots a live view of the arm for x seconds.
        %Input of any number other than zero runs for abs(x) seconds
        %Input of zero opens the plot permanently
        function livePlot(self, time)
            disp('in function');
            t0 = clock;
            if(abs(time) ~= 0)
                while etime(clock, t0) < time
                    disp('Time Duration');
                    f1 = figure(1);
                    currPos = self.measured_js(1,0);
                    %disp(currPos);
                    pos = zeros(3,1);
                    pos(1,1) = currPos(1,1);
                    pos(2,1) = currPos(1,2);
                    pos(3,1) = currPos(1,3);
                    %disp(pos);
                    clf;
                    view(3);
                    self.plot_arm(pos);
                end
            else
                while (true)
                    %disp('Infinite Loop');
                    f1 = figure(1);
                    currPos = self.measured_js(1,0);
                    %disp(currPos);
                    pos = zeros(3,1);
                    pos(1,1) = currPos(1,1);
                    pos(2,1) = currPos(1,2);
                    pos(3,1) = currPos(1,3);
                    %disp(pos);
                    clf;
                    view(3);
                    self.plot_arm(pos);
                end
            end
        end
        
        %Function takes in position vector, and identifies the corresponding joint angles
        function output = ik3001(self, posVector)
            try
                L0 = 55;                                    %set the inputs of lengths
                L1 = 40;
                L2 = 100;
                L3 = 100;
                
                %Ranges of the arms; still need to be properly found
                L1R = -9000:9000;
                L2R = -4500:10000;
                L3R = -9000:6300;
                
                %finding the length of the tip from the base
                r = sqrt(posVector(1)^2+posVector(2)^2 + (posVector(3)-95)^2);
                
                %helper equations to help find the angle measures
                h1 = acosd((L3^2-L2^2-r^2)/(-2*L2*r));
                h2 = atan2d((posVector(3)-95),(sqrt(posVector(1)^2+posVector(2)^2)));
                h3 = acosd((r^2-L2^2-L3^2)/(-2*L2*L3));
                
                %finding the angle measures
                q1 = atan2d(posVector(2),posVector(1));
                q2 = 90-h1-h2;
                q3 = 90-h3;
                
                %still need to implement the workable worspace once it is calculated
                
                backrange = false;
                if posVector(1) < 0 && posVector(3) < 237
                    disp('reading back');
                    backrange = true;
                end
                
                %Workspace Conditions:
                % 1) Valid Range of Motors
                % 2) Position is within 200mm of the arm
                % 3) If the arm is in the -x, it needs to be above a certain height so the arm can reach it
                if q1 >= -90 && q1 <= 90 && q2 >= -45 && q2 <= 100 && q3 >= -90 && q3 <= 63 && r <= 200 && backrange == false
                    output = [q1 q2 q3];
                    %disp(self.fk3001([q1;q2;q3],3));
                else
                    %disp('shutting down');
                    pp.shutdown();  %Shutdown robot
                end
                
            catch exception
                output = [];
                getReport(exception);
                disp('Command error, robot not able to reach position');
            end
        end
        
        %function that takes in a 3x1 matrix of the current joint angles and creates the corrosponding 6x3 jacobian matrix
        function output = jacob3001(self,q)
            %establishing link lengths
            L0 = 45;
            L1 = 50;
            L2 = 100;
            L3 = 100;
            
            %setting up joint angles 
            q = deg2rad(q);
            
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            a1 = -pi/2;   
            
            %hard coded computation for javobian outputs 
            w1 = [0;0;1];
            w2 = [sin(a1)*sin(q1); -sin(a1)*cos(q1); cos(a1)];
            w3 = w2;
            jacobMatrix = [L3*sin(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) - L3*cos(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)) - L2*cos(q2 - pi/2)*sin(q1) - L2*cos(a1)*cos(q1)*sin(q2 - pi/2), - L3*cos(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) - L3*sin(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2)) - L2*cos(q1)*sin(q2 - pi/2) - L2*cos(a1)*cos(q2 - pi/2)*sin(q1),- L3*cos(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) - L3*sin(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2) - cos(a1)*sin(q1)*sin(q2 - pi/2));
                L3*cos(q3 + pi/2)*(cos(q1)*cos(q2 - pi/2)- cos(a1)*sin(q1)*sin(q2 - pi/2)) - L3*sin(q3 + pi/2)*(cos(q1)*sin(q2 - pi/2) + cos(a1)*cos(q2 - pi/2)*sin(q1)) + L2*cos(q1)*cos(q2 - pi/2) - L2*cos(a1)*sin(q1)*sin(q2 - pi/2), L2*cos(a1)*cos(q1)*cos(q2 - pi/2) - L3*sin(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2)) - L2*sin(q1)*sin(q2 - pi/2) - L3*cos(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)),- L3*cos(q3 + pi/2)*(sin(q1)*sin(q2 - pi/2) - cos(a1)*cos(q1)*cos(q2 - pi/2)) - L3*sin(q3 + pi/2)*(cos(q2 - pi/2)*sin(q1) + cos(a1)*cos(q1)*sin(q2 - pi/2));
                0,L2*sin(a1)*cos(q2 - pi/2) + L3*sin(a1)*cos(q2 - pi/2)*cos(q3 + pi/2) - L3*sin(a1)*sin(q2 - pi/2)*sin(q3 + pi/2),L3*sin(a1)*cos(q2 - pi/2)*cos(q3 + pi/2) - L3*sin(a1)*sin(q2 - pi/2)*sin(q3 + pi/2);
                w1(1),w2(1),w3(1);
                w1(2),w2(2),w3(2);
                w1(3),w2(3),w3(3)];
            output = jacobMatrix;
        end
        
        %Function calculates the forward differential kinematics based on the current joint angles.
        %Inputs: configuration q: current joint angles, qdot: Instantaneous joint velocities. 
        function output = fdk3001(self, q, qdot)
            qdot = deg2rad(qdot);
            %qdot                        %Display qdot to make sure it read values
            Jacob = self.jacob3001(q);  %Determine the jacobian based on the input angles
            mult = Jacob*qdot;          %Multiply the jacobian by the qdot matrix to get the velocities 
            
            %convert angular velocities from rad/sec to deg/sec
            mult(4) = rad2deg(mult(4)); 
            mult(5) = rad2deg(mult(5));
            mult(6) = rad2deg(mult(6));
            
            output = mult;  %Set the matrix to the output
        end
        
        %Function takes in a 1x3 goal in task space coordinates and creates a 3x1 Unit vector pointing from the tip to the goal. 
        function unitVector = goalUnitVector(self,goal)
            currentAngles = self.measured_js(1,0);
            currentAngles = transpose(currentAngles(1,:));
            currPos = self.fk3001(currentAngles, 3);
            currPos = currPos((1:3),4);
            x = goal(1,1)-currPos(1,1);
            y = goal(1,2)-currPos(2,1);
            z = goal(1,3)-currPos(3,1);
            mag = sqrt((x^2)+(y^2)+(z^2));
            xUnit = x/mag;
            yUnit = y/mag;
            zUnit = z/mag;
            unitVector = [xUnit ;yUnit ;zUnit];
        end
        
        %Function inputs goal and desired speed coefficient and creates the instantaneous velocity vector
        %3x1 matrix represents p.
        function output = instantVelocityVector(self, goal, speedCoeff)
            output = self.goalUnitVector(goal) * speedCoeff;
        end
        
        %function that takes in the jacobian and returns the inverse of the top 3 rows
        function output = invJacob(self)
            jointAngles = self.measured_js(1,0);
            jointAngles = transpose(jointAngles(1,:));
            jacob = self.jacob3001(jointAngles);
            output = inv(jacob((1:3),:));
        end 
        
        %function that uses the inverse jacobian and the instant velocity vector and createsthe inv velocity kinematics
        function output = invVelKinematics(self, goal, speedCoeff)
            inverse = self.invJacob();
            velVect = self.instantVelocityVector(goal, speedCoeff);
            qDot = inverse * velVect;
            output = rad2deg(qDot);
        end   
        %function that implements the numberical inverse kinematics algorithm
        function ik_3001_numerical(self, q0)
            y=0;
            speedCoefficient = 100; % maybe change idk
            THRESH = 1;
            
            tipData = [];
            xData = [];
            yData = [];
            zData = [];
            timeData = [];
            
            currAngle = q0;
            
            figure(1);
            xlim([0 220]);
            ylim([0 295]);
            [x,z] = ginput(1);  %set the number of arbitrary points on xz
            pf = [x;y;z];       %Set the final point in task space
            
            atPoint = false;    %Flag for breaking the while loop
            inc = 1;
            tic;
            while atPoint == false
                % floor(inc/15) == inc/15
                    %Plot the arm as it moves
                    figure(2);
                    clf;
                    view(0,0);
                    self.plot_arm(currAngle);
                    plot3(x,y,z,'r*');
                    zlim([0 300]);
                    drawnow();
                %end
                
                %Determinethe task space position
                currPos = self.fk3001(currAngle,3);
                currPos = currPos((1:3),4);
                
                xData = cat(1,xData,currPos(1));
                yData = cat(1,yData,currPos(2));
                zData = cat(1,zData,currPos(3));
                timeData = cat(1,timeData,toc);
                
                %Create the unit vector pointing towards the goal based on the current position
                xErr = pf(1)-currPos(1);
                yErr = pf(2)-currPos(2);
                zErr = pf(3)-currPos(3);
                mag = sqrt((xErr^2)+(yErr^2)+(zErr^2));
                xUnit = xErr/mag;
                yUnit = yErr/mag;
                zUnit = zErr/mag;
                unitVector = [xUnit ;yUnit ;zUnit];
                
                %create instantaneous velocity vector based on a speed coefficient
                velVector = unitVector * speedCoefficient;
                
                %Calculate inverse jacbian based on current joint positions
                jacob = self.jacob3001(currAngle);
                invJacob = inv(jacob((1:3),:));
                
                %Create a vector of instantaneous joint velocities
                jointVel = invJacob * velVector;
                jointVel = rad2deg(jointVel);
                
                %Add the velocities to the current Angle as joint increments
                q1New = currAngle(1) + (jointVel(1)/100);
                q2New = currAngle(2) + (jointVel(2)/100);
                q3New = currAngle(3) + (jointVel(3)/100);
                nextAngle = [q1New; q2New; q3New];
                
                currAngle = nextAngle;  %Set the current Angle to the next calculated Angle
                
                %Calculate if the end effector is at the goal. 
                if abs(xErr) < THRESH && abs(yErr) < THRESH && abs(zErr) < THRESH
                    atPoint = true;
                    disp("Arm has reached the setpoint!");
                end
                
                inc = inc + 1;
                pause(.01);   
            end
            
            fkCompare = self.fk3001(currAngle,3);
            fkCompare = fkCompare((1:3),4);
            fkCompare
            
            figure(3);
            title('Position vs. Time');
            hold on;
            plot(timeData,xData);
            plot(timeData,zData);
            legend('X Position', 'Z Posision');
            ylabel('Robot Position (mm)')
            xlabel('Time (Seconds)');
        end
        
        function ik_3001_numerical_EC(self)
            speedCoefficient = 300; % maybe change idk
            THRESH = 3;
            
            tipData = [];
            xData = [];
            yData = [];
            zData = [];
            timeData = [];
            
            currAngle = self.measured_js(1,0);
            currAngle = transpose(currAngle(1,:));
            
            figure(1);
            xlim([0 200]);
            ylim([0 295]);
            [x,z] = ginput(6);  %set the number of arbitrary points on xz
            p=[];
            y = [0; 0; 0; 0; 0; 0];
            p = cat(2, p, x, y, z);
            p = cat(1, p, p(1,:));
            atPoint = false;    %Flag for breaking the while loop
            inc = 1;
            side = 1;
            tic;
            while side < size(p,1) + 1
                self.interpolate_jp(currAngle,10)
                if atPoint == false
                    %Plot the arm as it moves
                    figure(1);
                    clf;
                    view(0,0);
                    self.plot_arm(currAngle);
                    plot3(x,y,z,'r*');
                    zlim([0 300]);
                    drawnow();
                    
                    %Determinethe task space position
                    currPos = self.fk3001(currAngle,3);
                    currPos = currPos((1:3),4);
                    
                    xData = cat(1,xData,currPos(1));
                    yData = cat(1,yData,currPos(2));
                    zData = cat(1,zData,currPos(3));
                    timeData = cat(1,timeData,toc);
                    
                    %Create the unit vector pointing towards the goal based on the current position
                    xErr = p(side,1)-currPos(1);
                    yErr = p(side,2)-currPos(2);
                    zErr = p(side,3)-currPos(3);
                    mag = sqrt((xErr^2)+(yErr^2)+(zErr^2));
                    xUnit = xErr/mag;
                    yUnit = yErr/mag;
                    zUnit = zErr/mag;
                    unitVector = [xUnit ;yUnit ;zUnit];
                    
                    %create instantaneous velocity vector based on a speed coefficient
                    velVector = unitVector * speedCoefficient;
                    
                    %Calculate inverse jacbian based on current joint positions
                    jacob = self.jacob3001(currAngle);
                    invJacob = inv(jacob((1:3),:));
                    
                    %Create a vector of instantaneous joint velocities
                    jointVel = invJacob * velVector;
                    jointVel = rad2deg(jointVel);
                    
                    %Add the velocities to the current Angle as joint increments
                    q1New = currAngle(1) + (jointVel(1)/100);
                    q2New = currAngle(2) + (jointVel(2)/100);
                    q3New = currAngle(3) + (jointVel(3)/100);
                    nextAngle = [q1New; q2New; q3New];
                    
                    currAngle = nextAngle;  %Set the current Angle to the next calculated Angle
                    
                    %Calculate if the end effector is at the goal.
                    if abs(xErr) < THRESH && abs(yErr) < THRESH && abs(zErr) < THRESH
                        atPoint = true;
                        disp("Arm has reached the setpoint!");
                    end
                    
                    inc = inc + 1;
                    %pause(.01);
                    if atPoint == true
                        disp("Moving to the Next Side");
                        side = side + 1;
                        atPoint = false;
                    end
                end
            end
            
%             fkCompare = self.fk3001(currAngle,3);
%             fkCompare = fkCompare((1:3),4);
            
            figure(3);
            title('Position vs. Time');
            hold on;
            plot(timeData,xData);
            plot(timeData,zData);
            legend('X Position', 'Z Posision');
            ylabel('Robot Position (mm)')
            xlabel('Time (Seconds)');
        end
        
        %changing the speedCoefficent to be an input so that we can change the speed at which the robot travels
        %also inputing the coord pos of the ball (pf) into the eq so that it travels to the indicated ball position. 
        function ik_3001_final_num(self, pf, speedCoefficient)
            THRESH = 1;
            
            %tipData = [];
            xData = [];
            yData = [];
            zData = [];
            timeData = [];
            
            currAngle = self.measured_js(1,0);
            currAngle = transpose(currAngle(1,:));
            
            atPoint = false;    %Flag for breaking the while loop
            inc = 1;
            tic;
            while atPoint == false
                self.interpolate_jp(currAngle,10);
                
%                 figure(2);
%                 clf;
%                 view(0,0);
%                 self.plot_arm(currAngle);
%                 zlim([0 300]);
%                 drawnow();
              
                %Determinethe task space position
                currPos = self.fk3001(currAngle,3);
                currPos = currPos((1:3),4);
                
                xData = cat(1,xData,currPos(1));
                yData = cat(1,yData,currPos(2));
                zData = cat(1,zData,currPos(3));
                timeData = cat(1,timeData,toc);
                
                %Create the unit vector pointing towards the goal based on the current position
                xErr = pf(1)-currPos(1);
                yErr = pf(2)-currPos(2);
                zErr = pf(3)-currPos(3);
                mag = sqrt((xErr^2)+(yErr^2)+(zErr^2));
                xUnit = xErr/mag;
                yUnit = yErr/mag;
                zUnit = zErr/mag;
                unitVector = [xUnit ;yUnit ;zUnit];
                
                %create instantaneous velocity vector based on a speed coefficient
                velVector = unitVector * speedCoefficient;
                
                %Calculate inverse jacbian based on current joint positions
                jacob = self.jacob3001(currAngle);
                invJacob = inv(jacob((1:3),:));
                
                %Create a vector of instantaneous joint velocities
                jointVel = invJacob * velVector;
                jointVel = rad2deg(jointVel);
                
                %Add the velocities to the current Angle as joint increments
                q1New = currAngle(1) + (jointVel(1)/100);
                q2New = currAngle(2) + (jointVel(2)/100);
                q3New = currAngle(3) + (jointVel(3)/100);
                nextAngle = [q1New; q2New; q3New];
                
                currAngle = nextAngle;  %Set the current Angle to the next calculated Angle
                
                %Calculate if the end effector is at the goal. 
                if abs(xErr) < THRESH && abs(yErr) < THRESH && abs(zErr) < THRESH
                    atPoint = true;
                    %disp("Arm has reached the setpoint!");
                end
                
                inc = inc + 1;
                pause(.01);   
            end
%             fkCompare = self.fk3001(currAngle,3);
%             fkCompare = fkCompare((1:3),4);
%             fkCompare
            
%             figure(3);
%             title('Position vs. Time');
%             hold on;
%             plot(timeData,xData);
%             plot(timeData,zData);
%             legend('X Position', 'Z Posision');
%             ylabel('Robot Position (mm)')
%             xlabel('Time (Seconds)');
        end
        
        %function takes in the camCoords, ballOffset val, and moves the robot to the above the position
        function moveAbovePos(self, camCoord, speedCoefficient)
            %set position array
            posArray = [camCoord(1); camCoord(2); camCoord(3) + 25];
            
            %run current joint angle, indicated target Position, and speed Coeffiecient
            self.ik_3001_final_num(posArray, speedCoefficient);
        end
        
        %function that takes in the camCoord, ball offsets, and speed coeffiencients and moves slowly to the ball pos
        function moveToPickupPos(self, camCoord, speedCoeffiecient) %speed coeffiencient should be lower than previously to slow down the movement
            %create the new position array
            posArray = [camCoord(1); camCoord(2); camCoord(3)]; %set to move position of the ball. 
            %set the output of the arm to move based on the input values
            self.ik_3001_final_num(posArray, speedCoeffiecient);
        end
        
        
        % function that raises the arm so that it does not interfere with the other balls on the field 
        function raiseArm(self, camCoord, speedCoef)
            xVal = camCoord(1);
            yVal = camCoord(2);
            zVal = camCoord(3);
            
            %set position array
            posArray = [xVal; yVal; zVal+90]; % 80 indicates above position 80 coord
            
            %measure the current angle of the robot's joints
           
            
            %set the output of the arm to move based on the input values
             self.ik_3001_final_num(posArray, speedCoef);
        end
            
        % move to the Drop position based on the box position of the designated color being held
        function moveToDropPos(self, boxPos, speedCoefficient)
            
            %set the output of the arm to move based on the input values
            self.ik_3001_final_num(boxPos, speedCoefficient);
        end
        
    end
end
 
 
 

