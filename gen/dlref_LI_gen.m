function dlref_LI = dlref_LI_gen(in1)
%dlref_LI_gen
%    dlref_LI = dlref_LI_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 13:18:02

tLI = in1(2,:);
tl_min = in1(3,:);
ts = in1(1,:);
t2 = tLI.*3.898846518675137;
t3 = tl_min.*3.898846518675137;
t4 = ts.*3.898846518675137;
t5 = -t4;
t6 = t2+t3+t5;
dlref_LI = 1.0./cosh(t6).^3.*sinh(t6).*(-5.689606907956037e-1);
end
