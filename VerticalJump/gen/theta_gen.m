function theta = theta_gen(in1)
%THETA_GEN
%    THETA = THETA_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:37:51

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2;
t3 = q3+t2;
theta = atan((sin(q1).*2.69e+2+sin(t2).*2.4e+2+sin(t3).*1.65e+2)./(cos(q1).*2.69e+2+cos(t2).*2.4e+2+cos(t3).*1.65e+2));
end
