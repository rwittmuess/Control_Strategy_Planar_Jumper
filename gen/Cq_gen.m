function Cq = Cq_gen(in1)
%Cq_gen
%    Cq = Cq_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    09-May-2023 20:50:32

dq1 = in1(6,:);
dq2 = in1(7,:);
dq3 = in1(8,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = sin(q2);
t3 = sin(q3);
t4 = q1+q2;
t5 = q2+q3;
t6 = dq1.^2;
t7 = dq2.^2;
t8 = dq3.^2;
t9 = cos(t4);
t10 = q3+t4;
t11 = sin(t4);
t12 = sin(t5);
t15 = dq1.*dq3.*t3.*(9.9e+1./5.0e+1);
t16 = dq2.*dq3.*t3.*(9.9e+1./5.0e+1);
t19 = t3.*t8.*(9.9e+1./1.0e+2);
t13 = cos(t10);
t14 = sin(t10);
t17 = -t15;
t18 = -t16;
t20 = -t19;
t21 = t6.*t12.*(9.9e+1./1.0e+2);
mt1 = [t17+t18+t20-t2.*t7.*(3.6e+1./2.5e+1)-t7.*t12.*(9.9e+1./1.0e+2)-t8.*t12.*(9.9e+1./1.0e+2)-dq1.*dq2.*t2.*(7.2e+1./2.5e+1)-dq1.*dq2.*t12.*(9.9e+1./5.0e+1)-dq1.*dq3.*t12.*(9.9e+1./5.0e+1)-dq2.*dq3.*t12.*(9.9e+1./5.0e+1);t17+t18+t20+t21+t2.*t6.*(3.6e+1./2.5e+1);t21+t3.*t6.*(9.9e+1./1.0e+2)+t3.*t7.*(9.9e+1./1.0e+2)+dq1.*dq2.*t3.*(9.9e+1./5.0e+1)];
mt2 = [t6.*t9.*(-2.4e+1./5.0)-t7.*t9.*(2.4e+1./5.0)-t6.*t13.*(3.3e+1./1.0e+1)-t7.*t13.*(3.3e+1./1.0e+1)-t8.*t13.*(3.3e+1./1.0e+1)-t6.*cos(q1).*(2.69e+2./5.0e+1)-dq1.*dq2.*t9.*(4.8e+1./5.0)-dq1.*dq2.*t13.*(3.3e+1./5.0)-dq1.*dq3.*t13.*(3.3e+1./5.0)-dq2.*dq3.*t13.*(3.3e+1./5.0);t6.*t11.*(-2.4e+1./5.0)-t7.*t11.*(2.4e+1./5.0)-t6.*t14.*(3.3e+1./1.0e+1)-t7.*t14.*(3.3e+1./1.0e+1)-t8.*t14.*(3.3e+1./1.0e+1)-t6.*sin(q1).*(2.69e+2./5.0e+1)-dq1.*dq2.*t11.*(4.8e+1./5.0)-dq1.*dq2.*t14.*(3.3e+1./5.0)-dq1.*dq3.*t14.*(3.3e+1./5.0)-dq2.*dq3.*t14.*(3.3e+1./5.0)];
Cq = [mt1;mt2];
end
