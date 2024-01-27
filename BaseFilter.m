% BaseFilter - Abstract class for implementing digital filters
%
% This class serves as a base class for implementing digital filters.
% It includes properties and abstract methods that should be implemented
% by derived filter classes.
%
% Properties (Access = protected):
%   - Tc: Sampling time (cycle time)
%
% Methods (Access = public):
%   - BaseFilter: Constructor to create BaseFilter objects
%
% Abstract Methods (Access = public):
%   - initialize: Abstract method to initialize variables
%   - starting: Abstract method to set the filter to a steady-state condition
%   - step: Abstract method to compute the filter output given an input
%
% In MATLAB, the handle class is a special class that enables objects to be
% passed by reference rather than by value. Objects of classes derived from
% handle are reference objects. When you create a handle object, a handle
% to the object is returned, allowing multiple variables to refer to the 
% same object. This is in contrast to value objects, where variables are 
% copies of the original object.
%
% matlab.mixin.Heterogeneous is a MATLAB mixin class that can be used to 
% create heterogeneous arrays of objects. A heterogeneous array is an array
% that can contain objects of different types, as long as they are derived 
% from a common base class. This is useful in situations where you want to 
% store different types of objects in a single array.

classdef (Abstract)  BaseFilter < handle & matlab.mixin.Heterogeneous
    properties (Access = protected)
        Tc % Sampling time (cycle time)
    end
    
    methods (Access = public)
        % Constructor for BaseFilter class
        function obj = BaseFilter(Tc)
            % Validate that Tc is a scalar and greater than zero
            assert(isscalar(Tc), 'Sampling time (Tc) must be a scalar');
            assert(Tc > 0, 'Sampling time (Tc) must be greater than zero');
            
            % Assign the sampling time to the property
            obj.Tc = Tc;
        end
    end

    methods (Abstract, Access = public)
        % Abstract methods to be implemented in derived classes

        % Initialize variables
        obj = initialize(obj)
        
        % Initialize the filter to start with input=output in a
        % steady-state condition
        obj = starting(obj, input)

        % Compute output given an input
        output = step(obj, input)
    end
end
