classdef Evaluator < handle
    properties  (Access = protected)
        Tc; % Cycle time
        fkElastic;
        fkRigid;

        cartesian_distance_penalty=100;
        time_penalty=0.01;
        expected_total_time;
    end

    methods  (Access = public)
        function obj = Evaluator(Tc,fkElastic,fkRigid)
            obj.Tc=Tc;
            obj.fkElastic=fkElastic;
            obj.fkRigid=fkRigid;

            valueSet=[5.8 5.20 8.1];
            keySet = {'+scara_data/decagon.code' '+scara_data/diamond.code' '+scara_data/square.code'};
            obj.expected_total_time = containers.Map(keySet,valueSet);

        end


        function score=evaluate(obj,tests_data)
            for itest=1:length(tests_data)
                scores(itest,1)=obj.evaluateSingleTest(tests_data(itest));
            end
            scores=sort(scores);
            score=mean(scores(2:end-1));
        end
    end

    methods  (Access = protected)
        function score=evaluateSingleTest(obj,test_data)
            cartesian_poses=zeros(size(test_data.full_position,1),2);
            cartesian_poses_from_encoder=zeros(size(test_data.full_position,1),2);
            max_cartesian_distance=-1;

            idx1=find(test_data.time>3,1,'first');
            for idx=1:size(test_data.full_position,1)
                T=obj.fkElastic(test_data.full_position(idx,:)');
                cartesian_poses(idx,:)=T([1 3],4);

                T=obj.fkRigid(test_data.measured_signal(idx,1:2)');
                cartesian_poses_from_encoder(idx,:)=T([1 3],4);

                if idx<idx1
                    continue
                end
                distance=sqrt(...
                    (test_data.target_cartesian(:,1)-cartesian_poses(idx,1)).^2+...
                    (test_data.target_cartesian(:,2)-cartesian_poses(idx,2)).^2);
                if min(distance)>max_cartesian_distance
                    [max_cartesian_distance,i_max_dist2]=min(distance);
                    i_max_dist1=idx;
                end
            end

            figure
            plot(test_data.target_cartesian(:,1),test_data.target_cartesian(:,2),'--k')
            hold on
            plot(cartesian_poses_from_encoder(:,1),cartesian_poses_from_encoder(:,2),'--r')
            plot(cartesian_poses(:,1),cartesian_poses(:,2),'b')
            plot([cartesian_poses(i_max_dist1,1) test_data.target_cartesian(i_max_dist2,1)],...
                [cartesian_poses(i_max_dist1,2) test_data.target_cartesian(i_max_dist2,2)],'r')
            axis equal;
            grid on
            legend('Target Cartesian position','Cartesian position from encoder','Actual Cartesian position')
            
            total_time=test_data.time(end);

            score = 100 -  ...
                max(0,total_time-obj.expected_total_time(test_data.name))*obj.time_penalty - ...
                max_cartesian_distance*obj.cartesian_distance_penalty;

            title(sprintf('Score = %f',score))
        end
    end
end