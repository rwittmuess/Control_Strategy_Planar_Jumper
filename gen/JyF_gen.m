function JyF = JyF_gen(in1)
%JyF_gen
%    JyF = JyF_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    05-May-2023 12:31:13

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = cos(t4);
t6 = q3+t4;
t7 = sin(t4);
t10 = t2.*2.69e+2;
t11 = t3.*2.69e+2;
t8 = cos(t6);
t9 = sin(t6);
t12 = t5.*2.4e+2;
t13 = t7.*2.4e+2;
t14 = t8.*1.65e+2;
t15 = t9.*1.65e+2;
t16 = t10+t12+t14;
t17 = t11+t13+t15;
t18 = t17.^2;
t19 = 1.0./t16;
t20 = t19.^2;
t21 = t18.*t20;
t22 = t21+1.0;
t23 = 1.0./t22;
JyF = reshape([0.0,1.0,1.0,0.0,t23.*(t19.*(t12+t14)+t17.*t20.*(t13+t15)),1.0,0.0,t23.*(t14.*t19+t15.*t17.*t20),1.0],[3,3]);
end
