% design reference 
syms ts tLO tLI real
t = 0:0.01:1;
td_LO = 1;
g = 9.81;

% parameter take-off trajectory
k11 = 6.32;
k12 = 1.8;
k13 = -1.17;
k14 = 6.67;


lref_LO = simplify(k11 * tanh(k12 * (ts - td_LO)) - 0.5*g*ts^2 + k13*ts + k14);
dlref_LO = simplify(diff(lref_LO,ts));
ddlref_LO = simplify(diff(dlref_LO,ts));

lref_LO_fun = matlabFunction(lref_LO);
dlref_LO_fun = matlabFunction(dlref_LO);
ddlref_LO_fun = matlabFunction(ddlref_LO);

lref_LOv = lref_LO_fun(t);
dlref_LOv = dlref_LO_fun(t);
ddlref_LOv = ddlref_LO_fun(t);

figure(1)
plot(t,lref_LOv);
hold on;
grid on;
plot(t,dlref_LOv);
%plot(t,ddlref_LOv);
legend(["position", "velocity", "acceleration"]);

