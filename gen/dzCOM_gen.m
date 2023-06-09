function dzCOM = dzCOM_gen(in1)
%dzCOM_gen
%    dzCOM = dzCOM_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    09-May-2023 20:50:27

dq1 = in1(6,:);
dq2 = in1(7,:);
dq3 = in1(8,:);
dzF = in1(9,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2;
t3 = q3+t2;
t4 = sin(t2);
t5 = sin(t3);
t6 = t4.*(3.2e+1./1.25e+2);
t7 = t5.*(2.2e+1./1.25e+2);
dzCOM = dzF-dq2.*(t6+t7)-dq3.*t5.*(2.2e+1./1.25e+2)-dq1.*(t6+t7+sin(q1).*2.869333333333333e-1);
end
