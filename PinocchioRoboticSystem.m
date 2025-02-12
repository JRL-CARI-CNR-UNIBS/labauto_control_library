classdef PinocchioRoboticSystem < MechanicalSystem
    properties  (Access = protected)
        njoints
        n_controlled_joints
        controlled_joints
        inputToState
        stateToOutput
        payload
        tool_mass
        model
        data
        K
        D
        motor_inertia
        motor_viscous_term
        motor_coulomb_term
    end

    methods  (Access = public)
        function obj=PinocchioRoboticSystem(st,model_name)
            obj@MechanicalSystem(st); % uso il costruttore parente
            obj.model=py.pinocchio.buildModelFromUrdf(sprintf('+%s/model.urdf',model_name));

            obj.data=obj.model.createData();
            
            
            [obj.K,obj.D,obj.motor_inertia,obj.motor_viscous_term,obj.motor_coulomb_term,obj.umax,obj.x0,noise,obj.u]=eval(sprintf('%s.dataSimulation',model_name));

            n=double(obj.model.nq);
            obj.njoints=n;
            obj.stateToOutput=[zeros(n) eye(n) zeros(n,2*n);
                               zeros(n,3*n) eye(n)];
            
            obj.n_controlled_joints=obj.njoints;
            obj.num_input=obj.n_controlled_joints;
            obj.num_output=2*obj.n_controlled_joints;

            obj.order=obj.njoints*2;
            obj.u0=zeros(obj.n_controlled_joints,1);
            obj.u=obj.u0;
            

            obj.sigma_y=[noise(1)*ones(obj.n_controlled_joints,1); noise(2)*ones(obj.n_controlled_joints,1)];
            
            % initial inital control action to stay close to steady state.
            obj.x=obj.x0;
            
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

            ql = x((1:obj.njoints));  % pos link
            qm = x((1:obj.njoints)+obj.njoints); %pos motor
            qld= x((1:obj.njoints)+2*obj.njoints); %vel link
            qmd= x((1:obj.njoints)+3*obj.njoints); %vel link

            tau_elastic=obj.K*(qm-ql)+obj.D*(qmd-qld);

            u=max(min(obj.umax,u),-obj.umax);
            
            q_numpy=py.numpy.array(ql);
            qp_numpy=py.numpy.array(qld);
            qpp_numpy=py.numpy.array(0*ql);
            nonlinear_tau = double(py.pinocchio.rnea(obj.model, obj.data, q_numpy,qp_numpy,qpp_numpy))';
            inertia=double(py.pinocchio.crba(obj.model,obj.data,q_numpy));
            
            %tau = inertia*qldd + nonlinear(ql,qld) 
            qldd = inertia\(tau_elastic-nonlinear_tau);

            qmdd=obj.motor_inertia\(u-tau_elastic-obj.motor_viscous_term*qmd-obj.motor_coulomb_term*tanh(30*qmd));
            Dx=[qld;qmd;qldd;qmdd];
        end
        function y=outputFunction(obj)
            y=obj.stateToOutput*obj.x+obj.sigma_y.*randn(length(obj.sigma_y),1);
        end

    end
end