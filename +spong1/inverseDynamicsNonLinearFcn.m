function out1 = inverseDynamicsNonLinearFcn(in1,in2,payload)
%inverseDynamicsNonLinearFcn
%    OUT1 = inverseDynamicsNonLinearFcn(IN1,IN2,PAYLOAD)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Feb-2024 13:24:09

Dql1 = in2(1,:);
Dqm1 = in2(2,:);
ql1 = in1(1,:);
qm1 = in1(2,:);
t2 = Dql1.*2.0e+2;
t3 = ql1.*5.0e+5;
t4 = qm1.*5.0e+5;
out1 = [Dqm1.*-2.0e+2+t2+t3-t4+cos(ql1).*(payload+5.0).*(9.81e+2./2.0e+2);Dqm1.*2.03e+2-t2-t3+t4+tanh(Dqm1.*1.0e+2).*4.0];
