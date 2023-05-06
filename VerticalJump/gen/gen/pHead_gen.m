function pHead = pHead_gen(in1)
%pHead_gen
%    pHead = pHead_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:40:02

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
xF = in1(5,:);
zF = in1(4,:);
t2 = q1+q2;
t3 = q3+t2;
pHead = [xF+sin(q1).*(3.0./1.0e+1)+sin(t2).*(3.0./1.0e+1)+sin(t3).*(3.0./5.0);zF+cos(q1).*(3.0./1.0e+1)+cos(t2).*(3.0./1.0e+1)+cos(t3).*(3.0./5.0)];
end
