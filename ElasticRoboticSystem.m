classdef ElasticRoboticSystem < MechanicalSystem
    properties  (Access = protected)
        njoints
        n_controlled_joints
        controlled_joints
        inverseDynamicsNonLinearFcn
        inertiaFcn
        inputToState
        stateToOutput
        payload
        tool_mass
    end

    methods  (Access = public)
        function obj=ElasticRoboticSystem(model_name)
            
            st=1e-3;
            obj@MechanicalSystem(st); % uso il costruttore parente
            
            obj.inverseDynamicsNonLinearFcn=eval(sprintf('@%s.inverseDynamicsNonLinearFcn',model_name));
            obj.inertiaFcn=eval(sprintf('@%s.inertiaSimulation',model_name));
            obj.inputToState=eval(sprintf('%s.inputToStateSimulation',model_name));
            obj.stateToOutput=eval(sprintf('%s.stateToOutputSimulation',model_name));
            obj.controlled_joints=eval(sprintf('%s.controlledJointSimulation',model_name));
            

            obj.njoints=size(obj.stateToOutput,2)/2;
            obj.n_controlled_joints=size(obj.controlled_joints,2);
            obj.num_input=obj.n_controlled_joints;
            obj.num_output=2*obj.n_controlled_joints;

            obj.umax=eval(sprintf('%s.uMaxSimulation',model_name));
            obj.order=obj.njoints*2;
            obj.u=zeros(obj.n_controlled_joints,1);

            obj.x0=[eval(sprintf('%s.initialStateSimulation',model_name));zeros(obj.njoints,1)];
            obj.x=obj.x0;

            noise=eval(sprintf('%s.noiseSimulation',model_name));
            obj.sigma_y=[noise(1)*ones(obj.n_controlled_joints,1); noise(2)*ones(obj.n_controlled_joints,1)];
            
            % initial inital control action to stay close to steady state.
            tau_nonlinear=obj.inverseDynamicsNonLinearFcn(obj.x0(1:obj.njoints),obj.x0((1:obj.njoints)+obj.njoints),0);
            obj.u=tau_nonlinear(1:2:end)+tau_nonlinear(2:2:end);
        

            for idx=1:obj.n_controlled_joints
                obj.output_names{idx}=sprintf('position_%d',idx);
                obj.output_names{idx+obj.n_controlled_joints}=sprintf('velocity_%d',idx);
            end

            for idx=1:obj.n_controlled_joints
                obj.input_names{idx}=sprintf('torque_%d',idx);
            end

            obj.payload=0;
        end

        function full_y=fullJointPosition(obj)
            full_y=obj.x(1:obj.njoints);
        end

        function setPayload(obj,payload)
            obj.payload=payload;
        end
    end



    methods  (Access = protected)
        function Dx=stateFunction(obj,x,u,t)

            ql=x(1:obj.njoints);  % pos
            qld=x((1:obj.njoints)+obj.njoints); %vel
            u=max(min(obj.umax,u),-obj.umax);
            tau=obj.controlled_joints*u;

           
            
            %tau = inertia*qldd + nonlinear(ql,qld) 
            nonlinear_tau=obj.inverseDynamicsNonLinearFcn(ql,qld,obj.payload);
            inertia = obj.inertiaFcn(ql,obj.payload);
            qldd = inertia\(tau-nonlinear_tau);
            
            Dx=[qld;qldd];
        end
        function y=outputFunction(obj)
            y=obj.stateToOutput*obj.x+obj.sigma_y.*randn(length(obj.sigma_y),1);
        end

    end
end