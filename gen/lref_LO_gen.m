function lref_LO = lref_LO_gen(in1)
%lref_LO_gen
%    lref_LO = lref_LO_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 16:42:28

td_LO = in1(2,:);
ts = in1(1,:);
lref_LO = ts.*(-1.049479994516011)-tanh(td_LO.*1.873756913207563-ts.*1.873756913207563).*6.222514730875357-ts.^2.*(9.81e+2./2.0e+2)+6.645865261388582;
end
