function dlref_LO = dlref_LO_gen(ts)
%dlref_LO_gen
%    dlref_LO = dlref_LO_gen(TS)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    04-May-2023 22:37:52

dlref_LO = ts.*(-9.81e+2./1.0e+2)-tanh(ts.*(9.0./5.0)-9.0./5.0).^2.*1.1376e+1+1.0206e+1;
end
