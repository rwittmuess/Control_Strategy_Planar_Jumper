function D = D_gen(in1)
%D_gen
%    D = D_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 13:17:58

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = q1+q2;
t7 = q2+q3;
t8 = cos(t6);
t9 = cos(t7);
t10 = q3+t6;
t11 = sin(t6);
t14 = t3.*(3.6e+1./2.5e+1);
t17 = t4.*(9.9e+1./5.0e+1);
t18 = t2.*(2.69e+2./5.0e+1);
t19 = t5.*(2.69e+2./5.0e+1);
t21 = t4.*(9.9e+1./1.0e+2);
t12 = cos(t10);
t13 = sin(t10);
t15 = t8.*(2.4e+1./5.0);
t16 = t11.*(2.4e+1./5.0);
t22 = -t19;
t25 = t9.*(9.9e+1./1.0e+2);
t27 = t21+7.63e+2./5.0e+2;
t20 = -t16;
t23 = t12.*(3.3e+1./1.0e+1);
t24 = t13.*(3.3e+1./1.0e+1);
t31 = t25+t27;
t33 = t14+t17+t25+2.971;
t26 = -t24;
t28 = t15+t23;
t29 = t20+t26;
t30 = t18+t28;
t32 = t22+t29;
D = reshape([t3.*(7.2e+1./2.5e+1)+t9.*(9.9e+1./5.0e+1)+t17+4.5808,t33,t31,t32,t30,t33,t17+2.971,t27,t29,t28,t31,t27,7.63e+2./5.0e+2,t26,t23,t32,t29,t26,7.5e+1./4.0,0.0,t30,t28,t23,0.0,7.5e+1./4.0],[5,5]);
end
