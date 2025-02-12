classdef MechanicalSystem < handle
    properties  (Access = protected)
        x; % state vector
        x0; % initial value of the state vector
        st; % sampling periodi
        sigma_y=0; % noise standard deviation (if MIMO, it is a vector)
        t=0; % time
        umax=5; % maximum input
        u
        u0

        order=2; % system order
        num_output=1; % number of system outputs
        num_input=1; % number of system inputs
        scenario=1;

        output_names;
        input_names;
    end

    methods  (Access = public)

        % obj=MechanicalSystem(st)
        % create a mechanical model with sampling period st
        function obj=MechanicalSystem(st)
            obj.st=st;
            obj.x=zeros(2,1);
            obj.x0=zeros(2,1);
            obj.order=2;
            obj.num_input=1;
            obj.num_output=1;
            obj.u0=zeros(obj.num_input,1);

            for idx=1:obj.num_output
                obj.output_names{idx}=sprintf('output_%d',idx);
            end
            for idx=1:obj.num_input
                obj.input_names{idx}=sprintf('input_%d',idx);
            end
        end

        % (re)initialize the system to initial conditions
        function obj=initialize(obj)
            obj.x=obj.x0;
            obj.u=obj.u0;
            obj.t=0;
        end

        function obj=setScenario(obj,scenario)
            obj.scenario=scenario;
        end

        function obj=writeActuatorValue(obj,u)
            obj.u=u;
        end

        function u=readActuatorValue(obj)
            u=obj.u;
        end

        function y=readSensorValue(obj)
            y=obj.outputFunction();
        end



        % solve derivative(x)=f(x,u) during sampling period
        function obj=simulate(obj)
            usat=obj.saturationControlAction(obj.u);
            n=10;
            dt=obj.st/n;
            for idx=1:n
                obj.odeSolver(usat,dt,dt*idx);
            end
        end


        % return system sampling period
        function st=getSamplingPeriod(obj)
            st=obj.st;
        end

        % return number of system outputs
        function num_output=getOutputNumber(obj)
            num_output=obj.num_output;
        end

        % return number of system inputs
        function num_input=getInputNumber(obj)
            num_input=obj.num_input;
        end

        function umax=getUMax(obj)
            umax=obj.umax;
        end

        function output_names=getOutputName(obj)
             output_names=obj.output_names;
        end

        function input_names=getInputName(obj)
            input_names=obj.input_names;
        end

        % show system
        function obj=show(obj)
            fprintf('This system has %d outputs:\n',obj.num_output)
            for io=1:obj.num_output
                fprintf('- %s\n',obj.output_names{io})
            end

            fprintf('This system has %d inputs:\n',obj.num_input)
            for ii=1:obj.num_input
                fprintf('- %s, with maximum limit = %f\n',obj.input_names{ii},obj.umax(ii))
            end
        end

    end

    methods  (Access = protected)

        % derivative(x)=f(x,u)
        function Dx=stateFunction(obj,x,u,t)
            Dx=0;
        end

        % ouput=g(x,u)
        function y=outputFunction(obj)
            y=obj.x+obj.sigma_y.*randn(length(obj.sigma_y),1);
        end

        % ODE solver (using RK4)
        function odeSolver(obj,u,st,t)
            % Runge Kutta 4
            k_1 = obj.stateFunction(obj.x,u,t);
            k_2 = obj.stateFunction(obj.x+0.5*st*k_1,u,t);
            k_3 = obj.stateFunction(obj.x+0.5*st*k_2,u,t);
            k_4 = obj.stateFunction(obj.x+k_3*st,u,t);
            obj.x=obj.x+(1/6)*(k_1+2*k_2+2*k_3+k_4)*st;
        end

        % saturate control action
        function usat=saturationControlAction(obj,u)
            for iu=1:obj.num_input
                if (u(iu)>obj.umax(iu))
                    usat(iu,1)=obj.umax(iu);
                elseif (u(iu)<-obj.umax(iu))
                    usat(iu,1)=-obj.umax(iu);
                else
                    usat(iu,1)=u(iu);
                end
            end
        end
    end
end
