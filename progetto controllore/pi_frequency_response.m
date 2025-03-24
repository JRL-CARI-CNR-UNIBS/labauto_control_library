function controller=pi_frequency_response(controller_parameters)
% V tuning parameters vector: Ti=V(1);  Kp=V(2);

s=tf('s');
Kp=controller_parameters(2);
Ti=controller_parameters(1);
C=Kp*(1+1/(Ti*s));
controller=@(w)reshape(freqresp(C,w),length(w),1);
