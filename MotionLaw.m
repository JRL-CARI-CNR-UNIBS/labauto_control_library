% MotionLaw - Abstract class for implementing motion laws in control systems
%
% This class provides a framework for defining motion laws, which describe
% the evolution of positions, velocities, and accelerations over time. 
%
% The MotionLaw class is a handle class, which means that objects of this
% class are passed by reference. This allows for efficient updates of the
% object's properties, particularly in the context of control systems where
% the state evolves over time.
%
% Properties:
%   - y, Dy, DDy: Actual positions, velocities, and accelerations
%   - target_y, target_Dy, target_DDy: Target positions, velocities, and accelerations
%   - tolerance: Tolerance for reaching the target position
%   - max_Dy, max_DDy: Upper bounds for velocities and accelerations (symmetrical)
%   - ndof: Number of degrees of freedom
%   - Tc: Cycle time
%   - instruction_list: List of motion instructions
%   - depending_instructions: Flag to check if there are depending instructions
%   - s: Curvilinear abscissa
%   - time: Current time
%   - t_rest: Rest time between instructions
%
% Methods:
%   - MotionLaw: Constructor to create MotionLaw objects
%   - initialize: Initialize method to set initial values
%   - addInstructions: Method to add instructions to the instruction list
%   - dependingInstructions: Method to check if there are depending instructions
%   - computeMotionLaw: Method to compute the motion law based on the current position
%
% Protected Methods:
%   - decodeInstruction: Method to decode the next instruction from the instruction list
%   - updateFunction: Abstract method to be implemented in derived classes for updating the motion law
%   - computeMotionLawTime: Abstract method to be implemented in derived
%   classes for computing the motion law time intervales
%
% Abstract Class:
%   - MotionLaw is an abstract class, and it cannot be instantiated. It is meant
%     to be subclassed, and the abstract methods must be implemented in the
%     derived classes to define specific motion laws.
 
classdef (Abstract) MotionLaw < handle
    properties  (Access = protected)
        % Actual state variables
        y; % Actual positions
        Dy; % Actual velocities
        DDy; % Actual accelerations
        
        % Target state variables
        target_y; % Target positions
        target_Dy; % Target velocities
        target_DDy; % Target accelerations

        tolerance = 1e-6; % Tolerance for reaching the target position
        
        % Upper bounds for velocities and accelerations (symmetrical)
        max_Dy; % Upper bound velocities
        max_DDy; % Upper bound acceleration

        ndof; % Number of degrees of freedom
        Tc; % Cycle time
        instruction_list; % List of motion instructions

        depending_instructions; % Flag to check if there are depending instructions

        s; % Curvilinear abscissa (not used in the provided code)
        time; % Current time
        t_rest=0; % Rest time between instructions

        y0; % initial condition when the system is (re)initialized

    end
    methods  (Access = public)

        % Constructor for MotionLaw class
        function obj=MotionLaw(max_Dy,max_DDy,Tc)
            
            % Input validation
            assert(isvector(max_Dy));
            assert(isvector(max_DDy));

            assert(iscolumn(max_Dy));
            assert(iscolumn(max_DDy));

            assert(isnumeric(max_Dy));
            assert(isnumeric(max_DDy));
            
            assert(isnumeric(Tc));
            assert(isscalar(Tc));
            obj.Tc=Tc;

            obj.ndof=length(max_Dy);
            assert(length(max_DDy)==obj.ndof);
            
            obj.max_Dy=max_Dy;
            obj.max_DDy=max_DDy;

            % Initialize object
            obj.y0=zeros(obj.ndof,1);
            initialize(obj);
        end

        function obj=setInitialCondition(obj,y0)
            obj.y0=y0;
            obj.target_y=y0;
            obj.initialize()
        end

        % Initialize method to set initial values
        function obj=initialize(obj)
            obj.y=obj.y0;
            obj.Dy=zeros(obj.ndof,1);
            obj.DDy=zeros(obj.ndof,1); 

            obj.target_y=obj.y0;
            obj.target_Dy=zeros(obj.ndof,1);
            obj.target_DDy=zeros(obj.ndof,1); 

            obj.instruction_list=[];
            obj.depending_instructions=true;

        end

        % Method to add instructions to the instruction list
        function obj=addInstructions(obj,instructions)
            if isempty(obj.instruction_list)
                obj.instruction_list=instructions;
            else
                obj.instruction_list=[obj.instruction_list;instructions];
            end
        end

        % Method to check if there are depending instructions
        function result=dependingInstructions(obj)
            result=obj.depending_instructions;
        end
        
        % Method to compute the motion law based on the current position
        function [y,Dy,DDy]=computeMotionLaw(obj,measure)
            % If no measure is provided, use the current position
            if nargin<2
                y=obj.y;
            else
                y=measure;
            end
            % Check if the current position is within tolerance of the target position
            if min(abs(obj.target_y - y))<obj.tolerance
                % If in tolerance, go to the next instruction
                obj.decodeInstruction();
            end
            % Update motion law based on the current state
            updateFunction(obj);
            y=obj.y;
            Dy=obj.Dy;
            DDy=obj.DDy;
            obj.time=obj.time+obj.Tc;
        end
    end  
    methods  (Access = protected)
        
        % Method to decode the next instruction from the instruction list
        function obj=decodeInstruction(obj)
            % Check if there is remaining rest time
            if (obj.t_rest>0)
                obj.t_rest=obj.t_rest-obj.Tc;
                return
            end
            % Check if the instruction list is empty
            if isempty(obj.instruction_list)
                obj.depending_instructions=false;
                return
            end
            obj.depending_instructions=true;
            
            % Extract the next command from the instruction list
            cmd=obj.instruction_list(1);
            obj.instruction_list=obj.instruction_list(2:end);

            % Check and process the command
            if cmd.startsWith("move: ")
                % Move to a target position
                target=str2num(erase(cmd,"move: "));
                valid_cmd=isnumeric(target) && isvector(target) && ...
                    all(not(isnan(target)));
                
                if not(valid_cmd)
                    warning("<< %s >> is wrong, skip",cmd);
                    obj.decodeInstruction();
                end

                if isrow(target)
                    target=target';
                end

                obj.target_y=target;
                obj.target_Dy=zeros(obj.ndof,1);
                obj.target_DDy=zeros(obj.ndof,1);
                obj.computeMotionLawTime();
            elseif cmd.startsWith("pause: ")
                % Pause for a specified time
                target=str2num(erase(cmd,"pause: "));
                obj.t_rest=target;
                
            else
                % Unrecognized command, skip
                warning("<< %s >> is wrong, skip",cmd);
                obj.decodeInstruction();
            end
        end
    end

    methods (Abstract, Access = protected)
        % Abstract methods to be implemented in derived classes
        obj=updateFunction(obj)
        obj=computeMotionLawTime(obj)
    end
end
