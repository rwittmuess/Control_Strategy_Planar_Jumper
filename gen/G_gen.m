function G = G_gen(in1)
%G_gen
%    G = G_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    09-May-2023 20:50:32

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2;
t3 = q3+t2;
t4 = sin(t2);
t5 = sin(t3);
t6 = t4.*4.7088e+1;
t7 = -t6;
t8 = t5.*3.2373e+1;
t9 = -t8;
G = [t7+t9-sin(q1).*5.27778e+1;t7+t9;t9;1.839375e+2;0.0];
end
