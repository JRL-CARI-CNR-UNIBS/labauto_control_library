function out1 = fkSimulation(in1)
%fkSimulation
%    OUT1 = fkSimulation(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Feb-2024 13:24:10

ql1 = in1(1,:);
t2 = cos(ql1);
t3 = sin(ql1);
out1 = reshape([t2,0.0,t3,0.0,-t3,0.0,t2,0.0,0.0,-1.0,0.0,0.0,t2,0.0,t3,1.0],[4,4]);