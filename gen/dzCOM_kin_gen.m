function dzCOM_kin = dzCOM_kin_gen(in1)
%dzCOM_kin_gen
%    dzCOM_kin = dzCOM_kin_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 13:18:02

dl_LO = in1(5,:);
td_LO = in1(2,:);
ts = in1(1,:);
dzCOM_kin = dl_LO+td_LO.*(9.81e+2./1.0e+2)-ts.*(9.81e+2./1.0e+2);
end
