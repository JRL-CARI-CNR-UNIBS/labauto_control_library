function out1 = inertiaSimulation(in1,payload)
%inertiaSimulation
%    OUT1 = inertiaSimulation(IN1,PAYLOAD)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Feb-2024 13:23:56

ql2 = in1(2,:);
t2 = cos(ql2);
t3 = payload.*(9.0./1.6e+1);
out1 = reshape([payload.*(4.5e+1./1.6e+1)+t2.*2.7e+1+payload.*t2.*(9.0./4.0)+4.0175e+1,t3+t2.*(payload.*(3.0./4.0)+9.0).*(3.0./2.0)+1.43e+2./2.0e+1,0.0,0.0,(t2.*(3.0./2.0)+3.0./4.0).*(payload+1.2e+1).*(3.0./4.0)+2.0./5.0,t3+1.43e+2./2.0e+1,0.0,0.0,0.0,0.0,4.70182629866372e+1,0.0,0.0,0.0,0.0,5.005],[4,4]);
