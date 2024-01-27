% TrapezoidalMotionLaw - Class for implementing trapezoidal motion law
%
% This class is a subclass of the MotionLaw abstract class. It defines a
% trapezoidal motion law, which is a type of motion profile often used in
% control systems. The trapezoidal
% motion law consists of acceleration, constant velocity, and deceleration
% phases, resulting in a trapezoidal velocity profile.
%
% Properties:
%   - t_acc: Time duration of the acceleration phase
%   - t_cruise_vel: Time duration of the constant velocity phase
%   - t_dec: Time duration of the deceleration phase
%   - y_ini: Initial position
%   - acc: Acceleration during the acceleration phase
%   - dec: Deceleration during the deceleration phase
%   - cruise_vel: Constant velocity during the constant velocity phase
%   - total_time: Total time for the trapezoidal motion profile
%
% Methods:
%   - TrapezoidalMotionLaw: Constructor to create TrapezoidalMotionLaw objects
%
% Inherited Properties from MotionLaw:
%   - y, Dy, DDy: Actual and target positions, velocities, and accelerations
%   - tolerance: Tolerance for reaching the target position
%   - max_Dy, max_DDy: Upper bounds for velocities and accelerations
%   - ndof: Number of degrees of freedom
%   - Tc: Cycle time
%   - instruction_list: List of motion instructions
%   - depending_instructions: Flag to check if there are depending instructions
%   - s: Curvilinear abscissa
%   - time: Current time
%   - t_rest: Rest time between instructions
%
% Methods Inherited from MotionLaw:
%   - initialize: Initialize method to set initial values
%   - addInstructions: Method to add instructions to the instruction list
%   - dependingInstructions: Method to check if there are depending instructions
%   - computeMotionLaw: Method to compute the motion law based on the current position
%   - decodeInstruction: Method to decode the next instruction from the instruction list
%   - updateFunction: Abstract method to be implemented in derived classes for updating the motion law
%   - computeMotionLawTime: Abstract method to be implemented in derived classes for computing the motion law over time

classdef TrapezoidalMotionLaw < MotionLaw
    properties  (Access = protected)
        t_acc=0; % Time duration of the acceleration phase
        t_cruise_vel=0; % Time duration of the constant velocity phase
        t_dec=0; % Time duration of the deceleration phase
        y_ini % Initial position
        acc % Acceleration during the acceleration phase
        dec % Deceleration during the deceleration phase
        cruise_vel % Constant velocity during the constant velocity phase
        total_time=0; % Total time for the trapezoidal motion profile
    end
    methods  (Access = public)

        % Constructor for TrapezoidalMotionLaw class
        function obj=TrapezoidalMotionLaw(max_Dy,max_DDy,Tc)
            % Call the constructor of the superclass (MotionLaw)
            obj@MotionLaw(max_Dy,max_DDy,Tc);
        end
    end

    methods  (Access = protected)
        % Method to update the motion law based on the current time
        function obj=updateFunction(obj)

            if (obj.time<obj.t_acc)
                % Acceleration phase
                dt=obj.time;
                obj.DDy=obj.acc;
                obj.Dy=obj.acc*dt;
                obj.y=obj.y_ini+0.5*obj.acc*dt^2;

            elseif (obj.time<obj.t_acc+obj.t_cruise_vel)
                % Constant velocity phase
                dt=obj.time-obj.t_acc;
                obj.DDy=zeros(obj.ndof,1);
                obj.Dy=obj.cruise_vel;
                obj.y=obj.y_ini+...
                    0.5*obj.acc*obj.t_acc^2 + ...
                    obj.cruise_vel*dt;

            elseif  (obj.time<obj.t_acc+obj.t_cruise_vel+obj.t_dec)
                % Deceleration phase
                dt=obj.time-obj.t_acc-obj.t_cruise_vel;
                obj.DDy=obj.dec;
                obj.Dy=obj.cruise_vel+obj.dec*dt;
                obj.y=obj.y_ini+ ...
                    0.5*obj.acc*obj.t_acc^2 + ...
                    obj.cruise_vel*obj.t_cruise_vel + ...
                    obj.cruise_vel*dt +...
                    0.5*obj.dec*dt^2;

            else
                % Motion is complete, stay at the target position
                obj.y=obj.target_y;
                obj.Dy=zeros(obj.ndof,1);
                obj.DDy=zeros(obj.ndof,1);

            end

        end


        % Method to compute the trapezoidal motion law parameters
        function obj=computeMotionLawTime(obj)
            obj.time=0;

            % Check if the target position is already reached
            distance=abs(obj.target_y-obj.y);
            if max(distance)<obj.tolerance
                obj.t_acc=0;
                obj.t_cruise_vel=0;
                obj.t_dec=0;
                obj.total_time=0;
                obj.acc=zeros(obj.ndof,1);
                obj.dec=zeros(obj.ndof,1);
                obj.cruise_vel=zeros(obj.ndof,1);

                obj.y_ini = obj.target_y;
                return
            end
            obj.y_ini = obj.y;

            % Determine the direction of motion (positive or negative)
            direction=sign(obj.target_y-obj.y);

            % Calculate joint-specific acceleration and deceleration times
            t_acc_joints=obj.max_Dy./obj.max_DDy;
            t_dec_joints=t_acc_joints;

            % Calculate distances covered during acceleration and deceleration
            distance_during_acc=0.5*obj.max_Dy.*t_acc_joints;
            distance_during_dec=0.5*obj.max_Dy.*t_dec_joints;

            % Calculate the total distance covered during acceleration and deceleration
            distance_during_accdec=distance_during_acc+distance_during_dec;

            % Calculate joint-specific times for constant velocity phase
            t_cruise_vel_joints=(distance-distance_during_accdec)./obj.max_Dy;

            % Loop through joints to handle special cases (triangular motion)
            for idx=1:obj.ndof
                if (t_cruise_vel_joints(idx)<0)
                    % Triangular case: t_acc=t_dec
                    % s1=0.5*max_DDy*t_acc^2, v1=max_DDy*t_acc
                    % s2=v1*t_dec-0.5*max_DDy*t_dec^2
                    % s2=max_DDy*t_acc^2-0.5*max_DDy*t_dec^2
                    % s2=0.5*max_DDy*t_acc^2
                    % stot=s1+s2 = max_DDy*t_acc^2
                    % t_acc=syrt(stot/max_DDy)

                    t_acc_joints(idx)=sqrt(distance(idx)/obj.max_DDy(idx));
                    t_dec_joints(idx)=t_acc_joints(idx);
                    t_cruise_vel_joints(idx)=0;
                else
                    % Trapezoidal case
                    % t_acc=t_dec
                    % s1=0.5*acc*t_acc^2
                    % s2=acc*t_acc*t_cruise_vel
                    % s3=acc*t_acc*t_dec-0.5*acc*t_dec^2
                    % s3=s1
                    % stot=s1+s2+s3
                    % stot=acc*t_acc^2+acc*t_acc*t_cruise_vel
                    % stot=acc*t_acc*(t_acc+t_cruise_vel)
                end
            end

            % Calculate total time and find the worst-case joint
            t_tot=t_acc_joints+t_cruise_vel_joints+t_dec_joints;
            [obj.total_time,idx_worst_case]=max(t_tot);
            obj.t_acc=t_acc_joints(idx_worst_case);
            obj.t_cruise_vel=t_cruise_vel_joints(idx_worst_case);
            obj.t_dec=t_dec_joints(idx_worst_case);

            % Calculate acceleration, deceleration, and constant velocity
            % parameters based on the worst-case joint
            obj.acc=distance/(obj.t_acc)/(obj.t_acc+obj.t_cruise_vel);
            obj.dec=-obj.acc;
            obj.cruise_vel=obj.acc*obj.t_acc;

            % Apply direction to parameters
            obj.acc = obj.acc.*direction;
            obj.dec = obj.dec.*direction;
            obj.cruise_vel = obj.cruise_vel.*direction;

        end
    end
end
