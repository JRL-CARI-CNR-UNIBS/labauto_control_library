st=1e-3;
r=PinocchioRoboticSystem(st,'scara0');

%%
r.initialize
tic
clear measures
t=(0:st:2)';
for idx=1:length(t)
    r.writeActuatorValue([50;10]);
    measures(idx,:)=r.readSensorValue;
    r.simulate;
end
toc

plot(t,measures(:,1:2))
%%
prs=py.importlib.import_module('pinocchio_robotic_system');
numpy=py.importlib.import_module('numpy');

rpy=py.pinocchio_robotic_system.PinocchioRoboticSystem(st,'scara0');
rpy.initialize
tic
clear measures
t=(0:st:2)';
for idx=1:length(t)
    rpy.write_actuator_value(py.numpy.array([50;10]));
    measures(idx,:)=double(rpy.read_sensor_value);
    rpy.simulate;
end
toc

plot(t,measures(:,1:2))