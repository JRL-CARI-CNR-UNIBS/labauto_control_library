function out1 = fkSimulation(in1)
%fkSimulation
%    OUT1 = fkSimulation(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Feb-2024 13:24:28

ql1 = in1(1,:);
ql2 = in1(2,:);
ql3 = in1(3,:);
t2 = cos(ql3);
t3 = sin(ql3);
t4 = ql1+ql2;
t5 = cos(t4);
t6 = sin(t4);
out1 = reshape([t2.*t5,-t3,t2.*t6,0.0,-t3.*t5,-t2,-t3.*t6,0.0,t6,0.0,-t5,0.0,t5./2.0+cos(ql1)./2.0+(t2.*t5)./2.0,t3.*(-1.0./2.0)-1.0./1.0e+1,t6./2.0+sin(ql1)./2.0+(t2.*t6)./2.0,1.0],[4,4]);
