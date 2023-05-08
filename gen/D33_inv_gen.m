function D33_inv = D33_inv_gen(in1)
%D33_inv_gen
%    D33_inv = D33_inv_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    07-May-2023 22:56:49

q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = q2+q3;
t5 = q2.*2.0;
t6 = q3.*2.0;
t10 = -q3;
t7 = cos(t5);
t8 = cos(t6);
t9 = cos(t4);
t11 = q3+t4;
t12 = q2+t4;
t15 = q2+t10;
t16 = t4.*2.0;
t18 = t2.*1.8971e+4;
t21 = t2.*1.70739e+5;
t24 = t3.*7.34085e+5;
t25 = t2.*1.422825e+6;
t13 = cos(t11);
t14 = cos(t12);
t17 = cos(t15);
t19 = cos(t16);
t20 = t9.*7.975e+3;
t23 = -t21;
t26 = t8.*4.9005e+4;
t28 = t9.*5.98125e+5;
t34 = t8.*1.3868415e+7;
t37 = t7.*1.460808e+8;
t27 = t13.*5.445e+3;
t30 = t17.*7.92e+3;
t31 = t13.*4.9005e+4;
t33 = t13.*4.08375e+5;
t36 = t14.*5.94e+5;
t38 = t17.*5.94e+5;
t40 = t19.*4.08375e+5;
t29 = -t27;
t41 = -t40;
t42 = t23+t26+t31-1.71502e+5;
t44 = t34+t37+t40-1.96038691e+8;
t45 = 1.0./t44;
t47 = t42.*t45.*1.666666666666667e+3;
t49 = t45.*(t18-t20+t29+t30).*1.5e+4;
t51 = t45.*(t24+t25-t28-t33-t36+t38+t41+1.638754e+6).*2.0e+2;
t48 = -t47;
t50 = -t49;
D33_inv = reshape([t45.*(t26-1.71502e+5).*1.666666666666667e+3,t48,t50,t48,t45.*(t2.*-4.268475e+6+t8.*6.125625e+5+t13.*1.225125e+6+t19.*6.125625e+5-4.601906e+6).*(4.0e+2./3.0),t51,t50,t51,t45.*(t3.*-2.93634e+6+t7.*1.728e+6+t14.*2.376e+6+t19.*8.1675e+5-5.426443e+6).*1.0e+2],[3,3]);
end
