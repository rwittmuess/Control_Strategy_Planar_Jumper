function dtheta3 = dtheta3_gen(in1)
%DTHETA3_GEN
%    DTHETA3 = DTHETA3_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    05-May-2023 12:31:13

dq1 = in1(6,:);
dq2 = in1(7,:);
dq3 = in1(8,:);
dtheta3 = dq1+dq2+dq3;
end