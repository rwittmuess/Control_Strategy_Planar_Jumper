function l = l_gen(in1)
%L_GEN
%    L = L_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 13:18:01

q2 = in1(2,:);
q3 = in1(3,:);
l = sqrt(2.0).*sqrt(cos(q2+q3).*4.4385e+4+cos(q2).*6.456e+4+cos(q3).*3.96e+4+7.8593e+4).*1.066666666666667e-3;
end
