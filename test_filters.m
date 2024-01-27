clear all;close all;clc;

Tc=0.01;
t=(0:Tc:1)';
s=tf('s');

wn=10;
xi_zero=0.1;
xi_pole=0.7;

notch_tf=(s^2+2*xi_zero*wn*s+wn^2)/(s^2+2*xi_pole*wn*s+wn^2);
nf=NotchFilter(Tc,wn,xi_zero,xi_pole);
nf.starting(0);

for idx=1:length(t)
    y_object(idx,1)=nf.step(1);
end
y_tf=step(notch_tf,t);
figure
stairs(t,y_object)
hold
plot(t,y_tf)
title('NotchFilter')
grid on
legend('implementation','continous transfer function');

%%
time_constant=.1;
lp_transfer_function=1/(time_constant*s+1);

lp=FirstOrderLowPassFilter(Tc,time_constant);
lp.starting(0);
for idx=1:length(t)
    y_object(idx,1)=lp.step(1);
end
y_tf=step(lp_transfer_function,t);
figure
stairs(t,y_object)
hold
plot(t,y_tf)
title('FirstOrderLowPassFilter')
grid on
legend('implementation','continous transfer function');

%%

b=ones(50,1)/50;

fir_transfer_function=tf(b',[1 zeros(1,49)],Tc);

fir=FIRFilter(Tc,b);
fir.starting(0);
for idx=1:length(t)
    y_object(idx,1)=fir.step(1);
end

y_tf=step(fir_transfer_function,t);
figure
stairs(t,y_object)
hold
stairs(t,y_tf)
title('FirstOrderLowPassFilter')
grid on
legend('implementation','discrete transfer function');
