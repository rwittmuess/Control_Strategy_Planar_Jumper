function lref_LO = lref_LO_gen(ts)
%lref_LO_gen
%    lref_LO = lref_LO_gen(TS)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:40:19

lref_LO = ts.*(-1.17e+2./1.0e+2)+tanh(ts.*(9.0./5.0)-9.0./5.0).*(1.58e+2./2.5e+1)-ts.^2.*(9.81e+2./2.0e+2)+6.67e+2./1.0e+2;
end
