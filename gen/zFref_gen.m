function zFref = zFref_gen(in1)
%zFref_gen
%    zFref = zFref_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    09-May-2023 10:47:49

td_LO = in1(2,:);
ts = in1(1,:);
tz_Fmax = in1(3,:);
zFref = 1.0./cosh(td_LO.*2.024437246619655e+1-ts.*2.024437246619655e+1+tz_Fmax.*2.024437246619655e+1).^2.*3.75587019986575e-1-1.581977375159494e-3;
end
