function dzCOM_kin = dzCOM_kin_gen(in1)
%dzCOM_kin_gen
%    dzCOM_kin = dzCOM_kin_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    02-May-2023 10:32:57

dl_LO = in1(4,:);
tLO = in1(2,:);
ts = in1(1,:);
dzCOM_kin = dl_LO+tLO.*(9.81e+2./1.0e+2)-ts.*(9.81e+2./1.0e+2);