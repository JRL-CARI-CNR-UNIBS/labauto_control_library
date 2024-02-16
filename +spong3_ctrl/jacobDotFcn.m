function out1 = jacobDotFcn(in1,in2)
%jacobDotFcn
%    OUT1 = jacobDotFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Feb-2024 13:24:37

Dq1 = in2(1,:);
Dq2 = in2(2,:);
Dq3 = in2(3,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = q1+q2;
t9 = Dq2.^2;
t10 = Dq3.^2;
t11 = q1.*2.0;
t12 = q2.*2.0;
t13 = Dq1+Dq2;
t16 = -q3;
t18 = Dq1./2.0;
t19 = Dq2./2.0;
t14 = cos(t8);
t15 = sin(t8);
t17 = q1+t8;
t22 = t7./2.0;
t23 = t8.*2.0;
t24 = t8+t16;
t30 = Dq3.*t6.*t7.*(-1.0./2.0);
t34 = t18+t19;
t20 = cos(t17);
t21 = sin(t17);
t25 = cos(t23);
t26 = sin(t23);
t27 = cos(t24);
t28 = Dq3.*t3.*t22;
t29 = Dq3.*t6.*t22;
t31 = (Dq3.*t4.*t14)./2.0;
t32 = (Dq3.*t4.*t15)./2.0;
t33 = t22+1.0./1.0e+1;
t35 = t3.*t34;
t36 = t6.*t34;
t37 = t4.*t35;
t38 = t4.*t36;
t39 = t13.*t14.*t33;
t40 = t13.*t15.*t33;
t42 = Dq3.*t27.*t35;
t43 = Dq3.*t27.*t36;
t41 = -t40;
t44 = -t42;
t45 = -t43;
t46 = t32+t39;
t50 = t28+t36+t38;
t51 = t30+t35+t37;
t47 = t31+t41;
t48 = Dq2.*t46;
t49 = Dq2.*t47;
t52 = t45+t48;
t53 = t44+t49;
out1 = [t2.*(t42-t49)-t5.*(t43-t48)-Dq1.*(t2.*t18+t2.*t51-t5.*t50);t2.*(t43-t48)+t5.*(t42-t49)-Dq1.*(t5.*t18+t2.*t50+t5.*t51);Dq2.*Dq3.*t2.*t7.*(-1.0./2.0);t10./2.0+t9.*t20+(t10.*t25)./2.0-(Dq1.*Dq3)./2.0-(Dq2.*Dq3)./2.0+Dq1.*Dq2.*t20+Dq3.*t18.*t25+Dq3.*t19.*t25;t9.*t21+(t10.*t26)./2.0+Dq1.*Dq2.*t21+Dq3.*t18.*t26+Dq3.*t19.*t26;t10.*t15];