function Jy = Jy_gen(in1)
%Jy_gen
%    Jy = Jy_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:40:18

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = q1+q2;
t7 = q2+q3;
t15 = sqrt(2.0);
t8 = cos(t6);
t9 = cos(t7);
t10 = q3+t6;
t11 = sin(t6);
t12 = sin(t7);
t16 = t2.*2.69e+2;
t17 = t5.*2.69e+2;
t20 = t4.*3.96e+4;
t21 = t3.*6.456e+4;
t13 = cos(t10);
t14 = sin(t10);
t18 = t8.*2.4e+2;
t19 = t11.*2.4e+2;
t24 = t9.*4.4385e+4;
t25 = t12.*4.4385e+4;
t22 = t13.*1.65e+2;
t23 = t14.*1.65e+2;
t31 = t20+t21+t24+7.8593e+4;
t26 = t16+t18+t22;
t27 = t17+t19+t23;
t32 = 1.0./sqrt(t31);
t28 = t27.^2;
t29 = 1.0./t26;
t30 = t29.^2;
t33 = t28.*t30;
t34 = t33+1.0;
t35 = 1.0./t34;
Jy = reshape([0.0,1.0,1.0,t15.*t32.*(t25+sin(q2).*6.456e+4).*(-5.333333333333333e-4),t35.*(t29.*(t18+t22)+t27.*t30.*(t19+t23)),1.0,t15.*t32.*(t25+sin(q3).*3.96e+4).*(-5.333333333333333e-4),t35.*(t22.*t29+t23.*t27.*t30),1.0],[3,3]);
end
