function dlref_F = dlref_F_gen(in1)
%dlref_F_gen
%    dlref_F = dlref_F_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:40:19

dl_LO = in1(4,:);
tLO = in1(2,:);
ts = in1(1,:);
t2 = tLO.*2.5e+1;
t3 = ts.*2.5e+1;
t4 = -t3;
t5 = t2+t4+1.5e+1./8.0;
dlref_F = dl_LO+tLO.*(9.81e+2./1.0e+2)-ts.*(9.81e+2./1.0e+2)-1.0./cosh(t5).^3.*sinh(t5).*(1.3e+1./2.0);
end